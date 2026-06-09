# PLS916H 9×16 LED driver
#
# Protocol: SPI Mode 0 @ 1 MHz (MSBFIRST), write-only.
# Packet: 13-byte header | 144-byte data | 1-byte checksum | 4-byte tail
# Latch: ~3 µs pulse on SCK after packet (achieved by clocking out one byte at ~200 kHz).
#
# Data layout: 144 bytes, one byte per LED (0x00=off, 0xFF=full brightness).
# Physical LED-to-byte mapping is board-specific and not yet documented.
#
# !!IMPORTANT!!: On some boards with PLS916H, the VCC test point is NOT actually connected
# to VCC. You may need to manually connect 3.0–4.2 V to a capacitor near the chip.
# The Glasgow I/O voltage (-V) sets the logic level for SCK/DIN only.

import struct
import logging
import asyncio

from amaranth import *
from amaranth.lib import enum, wiring, stream, io, memory
from amaranth.lib.wiring import In, Out

from glasgow.support.logging import dump_hex
from glasgow.abstract import AbstractAssembly, GlasgowPin, ClockDivisor
from glasgow.applet import GlasgowAppletV2


__all__ = ["PLS916HInterface"]


FRAME_SIZE = 144

# Fixed packet framing, captured from logic analyzer
_HEADER = bytes([0x5A, 0xFF, 0x01, 0x5A, 0x24, 0x21, 0x3D, 0x01, 0x83, 0x5A, 0xFF, 0x02, 0x5B])
_TAIL   = bytes([0x5A, 0xFF, 0x04, 0x5D])


class _Command(enum.Enum, shape=8):
    """Commands sent from the host to the FPGA.

    Values are spaced to leave room for future animation/pattern commands.
    """
    WriteFrame = 0x01  # followed by 144 bytes of LED data
    SetAll     = 0x02  # followed by 1 byte (brightness applied to all LEDs)
    AllOff     = 0x03  # all LEDs off immediately
    AllOn      = 0x04  # all LEDs on (0xFF) immediately
    Sync       = 0x05  # synchronization barrier; replies with one byte when done
    SetLED     = 0x06  # followed by: index(1), brightness(1); updates one LED and retransmits

    # Animation commands (0x10–0x1F)
    Strobe     = 0x10  # followed by: brightness(1), period_ms(2 LE), duty_pct(1)
    Chase      = 0x11  # followed by: step_ms(2 LE)
    Stop       = 0x12  # halt any running animation

    # Reserved ranges for future use:
    #   0x20–0x2F  configuration (timing, latch mode)
    #   0x30–0x3F  readback / status (if ever needed)


class _Phase(enum.Enum, shape=3):
    """Tracks which packet section is being shifted out, so the shared shift engine
    knows where to return after completing a byte."""
    Header   = 0
    Data     = 1
    Checksum = 2
    Tail     = 3
    Latch    = 4


class _AnimMode(enum.Enum, shape=2):
    """Active animation mode."""
    Off    = 0
    Strobe = 1
    Chase  = 2


class PLS916HComponent(wiring.Component):
    """Gateware that serialises 144-byte LED frames to the PLS916H.

    Accepts a byte stream of commands from the host. For every frame written, the component
    shifts out the fixed header, 144 data bytes, an 8-bit checksum, and the fixed tail via
    SPI Mode 0 at the rate set by ``divisor``. After the packet, it generates a latch pulse
    by shifting one byte at a lower rate (~200 kHz at 48 MHz sys_clk).

    Supports on-FPGA animations (strobe, chase) that run autonomously until stopped.
    """
    i_stream: In(stream.Signature(8))
    o_stream: Out(stream.Signature(8))
    divisor:  In(16)

    def __init__(self, ports, *, latch_divisor, ms_cycles):
        self._ports = ports
        self._latch_divisor = latch_divisor
        self._ms_cycles = ms_cycles

        super().__init__()

    def elaborate(self, platform):
        m = Module()

        # I/O buffers for SCK and DIN (output-only, no readback)
        if self._ports.sck is None:
            self._ports.sck = io.SimulationPort("o", 1)
        if self._ports.din is None:
            self._ports.din = io.SimulationPort("o", 1)

        m.submodules.sck_buf = sck_buf = io.Buffer("o", self._ports.sck)
        m.submodules.din_buf = din_buf = io.Buffer("o", self._ports.din)

        # Shift register and bit counter
        shreg     = Signal(8)
        bit_count = Signal(range(8))
        sck_reg   = Signal()

        m.d.comb += [
            sck_buf.o.eq(sck_reg),
            din_buf.o.eq(shreg[7]),  # MSB first
        ]

        # Half-period timer
        timer   = Signal(16)
        cur_div = Signal(16)

        # Phase register
        phase = Signal(_Phase)

        # Constant ROMs for header and tail
        header_rom = Array([C(b, 8) for b in _HEADER])
        tail_rom   = Array([C(b, 8) for b in _TAIL])

        header_idx = Signal(range(len(_HEADER)))
        tail_idx   = Signal(range(len(_TAIL)))

        # Frame buffer (144 bytes, uses block RAM)
        m.submodules.frame_mem = frame_mem = memory.Memory(
            shape=8, depth=FRAME_SIZE, init=[0] * FRAME_SIZE)
        frame_wr = frame_mem.write_port()
        frame_rd = frame_mem.read_port()

        frame_idx  = Signal(range(FRAME_SIZE))
        checksum   = Signal(8)
        fill_value = Signal(8)

        # SetLED temporaries
        set_led_idx = Signal(range(FRAME_SIZE))
        set_led_val = Signal(8)

        # Latch divisor
        latch_div = Signal(16, init=self._latch_divisor)

        # ---- Animation state ----
        anim_mode = Signal(_AnimMode)

        # Strobe parameters
        strobe_brightness = Signal(8)
        strobe_period_ms  = Signal(16)  # total period in ms
        strobe_duty_ms    = Signal(16)  # on-time in ms (derived from period * duty_pct / 100)
        strobe_on         = Signal()    # current state: 1=on, 0=off

        # Chase parameters
        chase_step_ms   = Signal(16)  # ms per step
        chase_led_pos   = Signal(range(FRAME_SIZE))  # 0..143

        # Millisecond timer for animations
        ms_prescaler = Signal(range(self._ms_cycles))
        ms_counter   = Signal(16)  # counts down ms until next animation event

        # Byte index for multi-byte command reception
        cmd_idx = Signal(range(4))

        with m.FSM():
            # ---- Command dispatch ----

            with m.State("IDLE"):
                m.d.sync += sck_reg.eq(0)
                # If animation is active and no host command pending, run the animation
                with m.If((anim_mode != _AnimMode.Off)):
                    m.next = "ANIM-WAIT"
                with m.Else():
                    m.d.comb += self.i_stream.ready.eq(1)
                    with m.If(self.i_stream.valid):
                        with m.Switch(self.i_stream.payload):
                            with m.Case(_Command.WriteFrame):
                                m.d.sync += frame_idx.eq(0)
                                m.d.sync += checksum.eq(0)
                                m.next = "RECV-FRAME"
                            with m.Case(_Command.SetAll):
                                m.next = "RECV-BRIGHTNESS"
                            with m.Case(_Command.AllOff):
                                m.d.sync += frame_idx.eq(0)
                                m.d.sync += fill_value.eq(0x00)
                                m.next = "FILL-FRAME"
                            with m.Case(_Command.AllOn):
                                m.d.sync += frame_idx.eq(0)
                                m.d.sync += fill_value.eq(0xFF)
                                m.next = "FILL-FRAME"
                            with m.Case(_Command.SetLED):
                                m.d.sync += cmd_idx.eq(0)
                                m.next = "RECV-SET-LED"
                            with m.Case(_Command.Sync):
                                m.next = "SYNC"
                            with m.Case(_Command.Strobe):
                                m.d.sync += cmd_idx.eq(0)
                                m.next = "RECV-STROBE"
                            with m.Case(_Command.Chase):
                                m.d.sync += cmd_idx.eq(0)
                                m.next = "RECV-CHASE"
                            with m.Case(_Command.Stop):
                                m.d.sync += anim_mode.eq(_AnimMode.Off)

            # ---- Sync ----

            with m.State("SYNC"):
                m.d.comb += [
                    self.o_stream.payload.eq(0x00),
                    self.o_stream.valid.eq(1),
                ]
                with m.If(self.o_stream.ready):
                    m.next = "IDLE"

            # ---- SetLED: receive index(1) + brightness(1), update frame buffer, retransmit ----

            with m.State("RECV-SET-LED"):
                m.d.comb += self.i_stream.ready.eq(1)
                with m.If(self.i_stream.valid):
                    with m.Switch(cmd_idx):
                        with m.Case(0):
                            m.d.sync += set_led_idx.eq(self.i_stream.payload)
                            m.d.sync += cmd_idx.eq(1)
                        with m.Case(1):
                            m.d.sync += set_led_val.eq(self.i_stream.payload)
                            m.next = "SET-LED-READ-OLD"

            # Read old value at set_led_idx so we can update the checksum
            with m.State("SET-LED-READ-OLD"):
                m.d.comb += frame_rd.addr.eq(set_led_idx)
                m.next = "SET-LED-APPLY"

            with m.State("SET-LED-APPLY"):
                m.d.comb += frame_rd.addr.eq(set_led_idx)
                # Update checksum: subtract old value, add new value
                m.d.sync += checksum.eq((checksum - frame_rd.data + set_led_val)[:8])
                # Write new value
                m.d.comb += [
                    frame_wr.addr.eq(set_led_idx),
                    frame_wr.data.eq(set_led_val),
                    frame_wr.en.eq(1),
                ]
                m.d.sync += header_idx.eq(0)
                m.next = "SEND-HEADER-PREP"

            # ---- Receive strobe parameters: brightness(1), period_ms(2 LE), duty_pct(1) ----

            with m.State("RECV-STROBE"):
                m.d.comb += self.i_stream.ready.eq(1)
                with m.If(self.i_stream.valid):
                    with m.Switch(cmd_idx):
                        with m.Case(0):
                            m.d.sync += strobe_brightness.eq(self.i_stream.payload)
                        with m.Case(1):
                            m.d.sync += strobe_period_ms[:8].eq(self.i_stream.payload)
                        with m.Case(2):
                            m.d.sync += strobe_period_ms[8:].eq(self.i_stream.payload)
                        with m.Case(3):
                            # duty_pct → compute duty_ms = period * pct / 100
                            # We'll do this in a separate state since multiply is expensive
                            m.d.sync += fill_value.eq(self.i_stream.payload)  # temp store pct
                            m.d.sync += [
                                anim_mode.eq(_AnimMode.Strobe),
                                strobe_on.eq(1),
                                ms_prescaler.eq(self._ms_cycles - 1),
                            ]
                            m.next = "STROBE-CALC-DUTY"
                    with m.If(cmd_idx != 3):
                        m.d.sync += cmd_idx.eq(cmd_idx + 1)

            # Compute strobe_duty_ms = strobe_period_ms * duty / 128
            # Duty is on a 0–128 scale (64 ≈ 50%, 128 = 100%).
            with m.State("STROBE-CALC-DUTY"):
                m.d.sync += strobe_duty_ms.eq(
                    ((strobe_period_ms * fill_value) >> 7)[:16])
                m.d.sync += ms_counter.eq(
                    ((strobe_period_ms * fill_value) >> 7)[:16])
                m.next = "ANIM-PREP-FRAME"

            # ---- Receive chase parameters: step_ms(2 LE) ----

            with m.State("RECV-CHASE"):
                m.d.comb += self.i_stream.ready.eq(1)
                with m.If(self.i_stream.valid):
                    with m.Switch(cmd_idx):
                        with m.Case(0):
                            m.d.sync += chase_step_ms[:8].eq(self.i_stream.payload)
                        with m.Case(1):
                            m.d.sync += chase_step_ms[8:].eq(self.i_stream.payload)
                            m.d.sync += [
                                anim_mode.eq(_AnimMode.Chase),
                                chase_led_pos.eq(0),
                                ms_prescaler.eq(self._ms_cycles - 1),
                                ms_counter.eq(0),  # send first frame immediately
                            ]
                            m.next = "ANIM-PREP-FRAME"
                    with m.If(cmd_idx != 1):
                        m.d.sync += cmd_idx.eq(cmd_idx + 1)

            # ---- Animation engine ----

            # Wait for ms_counter to reach 0, then prepare next frame
            with m.State("ANIM-WAIT"):
                # Check for stop command from host while waiting
                m.d.comb += self.i_stream.ready.eq(1)
                with m.If(self.i_stream.valid & (self.i_stream.payload == _Command.Stop)):
                    m.d.sync += anim_mode.eq(_AnimMode.Off)
                    m.next = "IDLE"
                with m.Elif(self.i_stream.valid & (self.i_stream.payload == _Command.Sync)):
                    m.next = "SYNC"
                with m.Else():
                    # Millisecond prescaler
                    with m.If(ms_prescaler == 0):
                        m.d.sync += ms_prescaler.eq(self._ms_cycles - 1)
                        with m.If(ms_counter == 0):
                            m.next = "ANIM-PREP-FRAME"
                        with m.Else():
                            m.d.sync += ms_counter.eq(ms_counter - 1)
                    with m.Else():
                        m.d.sync += ms_prescaler.eq(ms_prescaler - 1)

            # Prepare the frame buffer for the current animation state
            with m.State("ANIM-PREP-FRAME"):
                m.d.sync += frame_idx.eq(0)
                with m.Switch(anim_mode):
                    with m.Case(_AnimMode.Strobe):
                        with m.If(strobe_on):
                            m.d.sync += fill_value.eq(strobe_brightness)
                        with m.Else():
                            m.d.sync += fill_value.eq(0x00)
                        m.next = "FILL-FRAME"
                    with m.Case(_AnimMode.Chase):
                        m.d.sync += fill_value.eq(0x00)
                        m.next = "FILL-FRAME"

            # After FILL-FRAME completes and the packet is transmitted (LATCH-NEXT),
            # we return here to set up the next animation cycle.
            with m.State("ANIM-POST-TX"):
                with m.Switch(anim_mode):
                    with m.Case(_AnimMode.Strobe):
                        # Toggle on/off state and set timer for next phase
                        m.d.sync += strobe_on.eq(~strobe_on)
                        with m.If(strobe_on):
                            # Was on, now going off: wait for (period - duty) ms
                            m.d.sync += ms_counter.eq(
                                (strobe_period_ms - strobe_duty_ms)[:16])
                        with m.Else():
                            # Was off, now going on: wait for duty ms
                            m.d.sync += ms_counter.eq(strobe_duty_ms)
                        m.d.sync += ms_prescaler.eq(self._ms_cycles - 1)
                        m.next = "ANIM-WAIT"
                    with m.Case(_AnimMode.Chase):
                        # Advance to next LED position
                        with m.If(chase_led_pos == FRAME_SIZE - 1):
                            m.d.sync += chase_led_pos.eq(0)
                        with m.Else():
                            m.d.sync += chase_led_pos.eq(chase_led_pos + 1)
                        m.d.sync += ms_counter.eq(chase_step_ms)
                        m.d.sync += ms_prescaler.eq(self._ms_cycles - 1)
                        m.next = "ANIM-WAIT"
                    with m.Default():
                        m.next = "IDLE"

            # ---- Frame loading ----

            with m.State("RECV-FRAME"):
                m.d.comb += self.i_stream.ready.eq(1)
                with m.If(self.i_stream.valid):
                    m.d.comb += [
                        frame_wr.addr.eq(frame_idx),
                        frame_wr.data.eq(self.i_stream.payload),
                        frame_wr.en.eq(1),
                    ]
                    m.d.sync += checksum.eq((checksum + self.i_stream.payload)[:8])
                    with m.If(frame_idx == FRAME_SIZE - 1):
                        m.d.sync += header_idx.eq(0)
                        m.next = "SEND-HEADER-PREP"
                    with m.Else():
                        m.d.sync += frame_idx.eq(frame_idx + 1)

            with m.State("RECV-BRIGHTNESS"):
                m.d.comb += self.i_stream.ready.eq(1)
                with m.If(self.i_stream.valid):
                    m.d.sync += [
                        frame_idx.eq(0),
                        fill_value.eq(self.i_stream.payload),
                    ]
                    m.next = "FILL-FRAME"

            with m.State("FILL-FRAME"):
                m.d.comb += [
                    frame_wr.addr.eq(frame_idx),
                    frame_wr.data.eq(fill_value),
                    frame_wr.en.eq(1),
                ]
                with m.If(frame_idx == FRAME_SIZE - 1):
                    # For chase mode, after filling with zeros we set the one active LED
                    with m.If(anim_mode == _AnimMode.Chase):
                        m.next = "CHASE-SET-LED"
                    with m.Else():
                        m.d.sync += checksum.eq((fill_value * FRAME_SIZE)[:8])
                        m.d.sync += header_idx.eq(0)
                        m.next = "SEND-HEADER-PREP"
                with m.Else():
                    m.d.sync += frame_idx.eq(frame_idx + 1)

            # Set the one active LED for chase mode (full brightness at chase_led_pos)
            with m.State("CHASE-SET-LED"):
                m.d.comb += [
                    frame_wr.addr.eq(chase_led_pos),
                    frame_wr.data.eq(0xFF),
                    frame_wr.en.eq(1),
                ]
                # Checksum: 0xFF (one LED at full brightness, rest zero)
                m.d.sync += checksum.eq(0xFF)
                m.d.sync += header_idx.eq(0)
                m.next = "SEND-HEADER-PREP"

            # ---- Packet transmission ----

            # -- Header --
            with m.State("SEND-HEADER-PREP"):
                m.d.sync += [
                    shreg.eq(header_rom[header_idx]),
                    bit_count.eq(7),
                    cur_div.eq(self.divisor),
                    timer.eq(self.divisor),
                    phase.eq(_Phase.Header),
                ]
                m.next = "SHIFT-HIGH"

            with m.State("SEND-HEADER-NEXT"):
                with m.If(header_idx == len(_HEADER) - 1):
                    m.d.sync += frame_idx.eq(0)
                    m.next = "SEND-DATA-PREP"
                with m.Else():
                    m.d.sync += header_idx.eq(header_idx + 1)
                    m.next = "SEND-HEADER-PREP"

            # -- Data --
            with m.State("SEND-DATA-PREP"):
                m.d.comb += frame_rd.addr.eq(frame_idx)
                m.next = "SEND-DATA-LOAD"

            with m.State("SEND-DATA-LOAD"):
                m.d.comb += frame_rd.addr.eq(frame_idx)
                m.d.sync += [
                    shreg.eq(frame_rd.data),
                    bit_count.eq(7),
                    cur_div.eq(self.divisor),
                    timer.eq(self.divisor),
                    phase.eq(_Phase.Data),
                ]
                m.next = "SHIFT-HIGH"

            with m.State("SEND-DATA-NEXT"):
                with m.If(frame_idx == FRAME_SIZE - 1):
                    m.next = "SEND-CHECKSUM-PREP"
                with m.Else():
                    m.d.sync += frame_idx.eq(frame_idx + 1)
                    m.next = "SEND-DATA-PREP"

            # -- Checksum --
            with m.State("SEND-CHECKSUM-PREP"):
                m.d.sync += [
                    shreg.eq(checksum),
                    bit_count.eq(7),
                    cur_div.eq(self.divisor),
                    timer.eq(self.divisor),
                    tail_idx.eq(0),
                    phase.eq(_Phase.Checksum),
                ]
                m.next = "SHIFT-HIGH"

            with m.State("SEND-CHECKSUM-NEXT"):
                m.next = "SEND-TAIL-PREP"

            # -- Tail --
            with m.State("SEND-TAIL-PREP"):
                m.d.sync += [
                    shreg.eq(tail_rom[tail_idx]),
                    bit_count.eq(7),
                    cur_div.eq(self.divisor),
                    timer.eq(self.divisor),
                    phase.eq(_Phase.Tail),
                ]
                m.next = "SHIFT-HIGH"

            with m.State("SEND-TAIL-NEXT"):
                with m.If(tail_idx == len(_TAIL) - 1):
                    m.next = "LATCH-PREP"
                with m.Else():
                    m.d.sync += tail_idx.eq(tail_idx + 1)
                    m.next = "SEND-TAIL-PREP"

            # -- Latch pulse --
            with m.State("LATCH-PREP"):
                m.d.sync += [
                    shreg.eq(0x00),
                    bit_count.eq(7),
                    cur_div.eq(latch_div),
                    timer.eq(latch_div),
                    phase.eq(_Phase.Latch),
                ]
                m.next = "SHIFT-HIGH"

            with m.State("LATCH-NEXT"):
                m.d.sync += sck_reg.eq(0)
                # If animation is running, go to post-tx handler; otherwise back to idle
                with m.If(anim_mode != _AnimMode.Off):
                    m.next = "ANIM-POST-TX"
                with m.Else():
                    m.next = "IDLE"

            # ---- Shared bit-level shift engine ----

            with m.State("SHIFT-HIGH"):
                with m.If(timer == 0):
                    m.d.sync += [
                        sck_reg.eq(1),
                        timer.eq(cur_div),
                    ]
                    m.next = "SHIFT-LOW"
                with m.Else():
                    m.d.sync += timer.eq(timer - 1)

            with m.State("SHIFT-LOW"):
                with m.If(timer == 0):
                    m.d.sync += sck_reg.eq(0)
                    with m.If(bit_count == 0):
                        with m.Switch(phase):
                            with m.Case(_Phase.Header):
                                m.next = "SEND-HEADER-NEXT"
                            with m.Case(_Phase.Data):
                                m.next = "SEND-DATA-NEXT"
                            with m.Case(_Phase.Checksum):
                                m.next = "SEND-CHECKSUM-NEXT"
                            with m.Case(_Phase.Tail):
                                m.next = "SEND-TAIL-NEXT"
                            with m.Case(_Phase.Latch):
                                m.next = "LATCH-NEXT"
                    with m.Else():
                        m.d.sync += [
                            shreg.eq(shreg << 1),
                            bit_count.eq(bit_count - 1),
                            timer.eq(cur_div),
                        ]
                        m.next = "SHIFT-HIGH"
                with m.Else():
                    m.d.sync += timer.eq(timer - 1)

        return m


class PLS916HInterface:
    def __init__(self, logger: logging.Logger, assembly: AbstractAssembly, *,
                 sck: GlasgowPin, din: GlasgowPin):
        self._logger = logger
        self._level  = logging.DEBUG if self._logger.name == __name__ else logging.TRACE

        ports = assembly.add_port_group(sck=sck, din=din)

        sys_freq_hz = 1 / assembly.sys_clk_period
        ms_cycles = int(sys_freq_hz / 1000)

        # Latch divisor: target ~200 kHz
        latch_target_hz = 200_000
        latch_divisor = max(0, int(sys_freq_hz / (2 * latch_target_hz)) - 1)

        component = assembly.add_submodule(PLS916HComponent(ports,
            latch_divisor=latch_divisor, ms_cycles=ms_cycles))
        self._pipe = assembly.add_inout_pipe(component.o_stream, component.i_stream)
        self._clock = assembly.add_clock_divisor(component.divisor,
            ref_period=assembly.sys_clk_period, name="sck")

    def _log(self, message, *args):
        self._logger.log(self._level, "PLS916H: " + message, *args)

    @property
    def clock(self) -> ClockDivisor:
        """SCK clock divisor."""
        return self._clock

    async def _synchronize(self):
        """Wait until all previously submitted commands have been fully transmitted."""
        await self._pipe.send(struct.pack("<B", _Command.Sync.value))
        await self._pipe.flush()
        await self._pipe.recv(1)

    async def write_frame(self, data: bytes):
        """Write a raw 144-byte frame to the PLS916H."""
        if len(data) != FRAME_SIZE:
            raise ValueError(f"frame must be exactly {FRAME_SIZE} bytes, got {len(data)}")
        self._log("write frame=<%s>", dump_hex(data))
        await self._pipe.send(struct.pack("<B", _Command.WriteFrame.value) + data)
        await self._synchronize()

    async def set_brightness(self, brightness: int):
        """Set all LEDs to the same brightness (0–255)."""
        if not 0 <= brightness <= 255:
            raise ValueError(f"brightness must be 0–255, got {brightness}")
        self._log("set brightness=%d", brightness)
        await self._pipe.send(struct.pack("<BB", _Command.SetAll.value, brightness))
        await self._synchronize()

    async def all_off(self):
        """Turn all LEDs off immediately."""
        self._log("all off")
        await self._pipe.send(struct.pack("<B", _Command.AllOff.value))
        await self._synchronize()

    async def all_on(self):
        """Turn all LEDs on at full brightness immediately."""
        self._log("all on")
        await self._pipe.send(struct.pack("<B", _Command.AllOn.value))
        await self._synchronize()

    async def set_led(self, index: int, brightness: int):
        """Set a single LED's brightness and retransmit the frame.

        The frame buffer is persistent — other LEDs retain their previous values.

        Args:
            index: LED index (0–143).
            brightness: Brightness (0–255).
        """
        if not 0 <= index < FRAME_SIZE:
            raise ValueError(f"index must be 0–{FRAME_SIZE - 1}, got {index}")
        if not 0 <= brightness <= 255:
            raise ValueError(f"brightness must be 0–255, got {brightness}")
        self._log("set led=%d brightness=%d", index, brightness)
        await self._pipe.send(struct.pack("<BBB",
            _Command.SetLED.value, index, brightness))
        await self._synchronize()

    async def strobe(self, brightness: int, period_ms: int, duty: int = 64):
        """Start strobe animation.

        Args:
            brightness: LED brightness during on-phase (0–255).
            period_ms: Total strobe period in milliseconds (1–65535).
            duty: Duty cycle, 0–128 scale (64 ≈ 50%, 128 = 100%).
        """
        self._log("strobe brightness=%d period=%dms duty=%d/128", brightness, period_ms, duty)
        await self._pipe.send(struct.pack("<BBHB",
            _Command.Strobe.value, brightness, period_ms, duty))
        await self._pipe.flush()

    async def chase(self, step_ms: int):
        """Start chase animation (one LED at a time, cycling through all positions).

        Args:
            step_ms: Milliseconds per step (1–65535).
        """
        self._log("chase step=%dms", step_ms)
        await self._pipe.send(struct.pack("<BH",
            _Command.Chase.value, step_ms))
        await self._pipe.flush()

    async def stop(self):
        """Stop any running animation."""
        self._log("stop")
        await self._pipe.send(struct.pack("<B", _Command.Stop.value))
        await self._synchronize()


class DisplayPLS916HApplet(GlasgowAppletV2):
    preview = True
    logger = logging.getLogger(__name__)
    help = "control PLS916H 9×16 LED driver"
    description = """
    Control a PLS916H 9×16 LED matrix driver over its SPI-like serial interface.

    The PLS916H uses a quasi-SPI protocol (Mode 0, MSB first) with a fixed packet structure
    (13-byte header, 144-byte data payload, 8-bit checksum, 4-byte tail) followed by a ~3 µs
    latch pulse on the clock line.

    Connect SCK to the PLS916H CLK pin and DIN to the PLS916H DIN pin.

    *Important*: on some boards the VCC test point is not actually connected to VCC.
    You may need to manually supply 3.0–4.2 V to a capacitor near the chip.
    The ``-V`` voltage only sets the logic level for SCK/DIN.

    Turn all LEDs on:

    ::

        glasgow run display-pls916h -V 3.3 on

    Strobe at 10 Hz (100 ms period), 50% duty:

    ::

        glasgow run display-pls916h -V 3.3 strobe --period 100 --duty 64

    Chase pattern at 50 ms per step:

    ::

        glasgow run display-pls916h -V 3.3 chase --speed 50
    """
    required_revision = "C0"

    @classmethod
    def add_build_arguments(cls, parser, access):
        access.add_voltage_argument(parser)
        access.add_pins_argument(parser, "sck", default=True, required=True)
        access.add_pins_argument(parser, "din", default=True, required=True)

    def build(self, args):
        with self.assembly.add_applet(self):
            self.assembly.use_voltage(args.voltage)
            self.pls916h = PLS916HInterface(self.logger, self.assembly,
                sck=args.sck, din=args.din)

    @classmethod
    def add_setup_arguments(cls, parser):
        parser.add_argument(
            "-f", "--frequency", metavar="FREQ", type=int, default=1000,
            help="set SCK frequency to FREQ kHz (default: %(default)s)")

    async def setup(self, args):
        await self.pls916h.clock.set_frequency(args.frequency * 1000)

    @classmethod
    def add_run_arguments(cls, parser):
        sub = parser.add_subparsers(dest="command", required=True)

        sub.add_parser("on", help="all LEDs on at full brightness")
        sub.add_parser("off", help="all LEDs off")

        p_brightness = sub.add_parser("brightness", help="set uniform brightness")
        p_brightness.add_argument(
            "value", type=int,
            help="brightness value (0–255)")

        p_frame = sub.add_parser("frame", help="write a raw 144-byte frame")
        p_frame.add_argument(
            "data", type=bytes.fromhex,
            help="288 hex characters (144 bytes)")

        p_led = sub.add_parser("led", help="set a single LED's brightness")
        p_led.add_argument(
            "index", type=int,
            help="LED index (0–143)")
        p_led.add_argument(
            "value", type=int,
            help="brightness (0–255)")

        p_strobe = sub.add_parser("strobe", help="strobe all LEDs")
        p_strobe.add_argument(
            "--brightness", type=int, default=255,
            help="LED brightness during on-phase (0–255, default: 255)")
        p_strobe.add_argument(
            "--period", type=int, default=100,
            help="strobe period in milliseconds (default: 100)")
        p_strobe.add_argument(
            "--duty", type=int, default=64,
            help="duty cycle 0–128 (64 ≈ 50%%, default: 64)")

        p_chase = sub.add_parser("chase", help="chase pattern (one LED at a time)")
        p_chase.add_argument(
            "--speed", type=int, default=100,
            help="milliseconds per step (default: 100)")

        sub.add_parser("stop", help="stop any running animation")

    async def run(self, args):
        iface = self.pls916h
        try:
            match args.command:
                case "on":
                    await iface.all_on()
                case "off":
                    await iface.all_off()
                case "brightness":
                    await iface.set_brightness(args.value)
                case "frame":
                    await iface.write_frame(args.data)
                case "led":
                    await iface.set_led(args.index, args.value)
                case "strobe":
                    await iface.strobe(args.brightness, args.period, args.duty)
                case "chase":
                    await iface.chase(args.speed)
                case "stop":
                    await iface.stop()
                    return
            await asyncio.Event().wait()
        except asyncio.CancelledError:
            await iface.stop()
            await iface.all_off()

    @classmethod
    def tests(cls):
        from . import test
        return test.DisplayPLS916HAppletTestCase
