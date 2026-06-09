from glasgow.applet import GlasgowAppletV2TestCase, synthesis_test
from . import DisplayPLS916HApplet


class DisplayPLS916HAppletTestCase(GlasgowAppletV2TestCase, applet=DisplayPLS916HApplet):
    @synthesis_test
    def test_build(self):
        self.assertBuilds()
