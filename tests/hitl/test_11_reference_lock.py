"""LionDriver HIL test: the safety-mode lock of the reference build (TSR-512/513).

Verifies SWSR-512, SWSR-512a and SWSR-513 on the device (LionDriver WP-V-05). Needs:
  - a panda flashed with a reference build: LIONDRIVER_REFERENCE=1 scons
    (or the reference release build, RELEASE=1 LIONDRIVER_REFERENCE=1)
  - HITL=1 REFERENCE_BUILD=1 in the environment

Each refused request is checked through the health packet: the state it would change stays as it was.
"""
import os
import time
import unittest

from opendbc.car.structs import CarParams
from panda.tests.hitl.base import SPEED_NORMAL, PandaTestCase

SILENT = CarParams.SafetyModel.silent
NOOUTPUT = CarParams.SafetyModel.noOutput
ELM327 = CarParams.SafetyModel.elm327
TOYOTA = CarParams.SafetyModel.toyota
HONDA = CarParams.SafetyModel.hondaNidec
REFERENCE_PARAM = 73
SETTLE_S = 0.1
OTHER_SPEED = 250  # kbps; the bus runs at SPEED_NORMAL


@unittest.skipUnless(os.environ.get("REFERENCE_BUILD") == "1", "needs a LIONDRIVER_REFERENCE=1 panda build")
class TestReferenceLock(PandaTestCase):
  def _set_mode(self, mode, param=0):
    self.p.set_safety_mode(mode, param)
    self.p.send_heartbeat(engaged=False)
    time.sleep(SETTLE_S)
    return self.p.health()

  def setUp(self):
    super().setUp()
    h = self._set_mode(SILENT)
    self.assertEqual(h["safety_mode"], int(SILENT))

  def tearDown(self):
    self._set_mode(NOOUTPUT)
    self._set_mode(SILENT)
    super().tearDown()

  def test_only_reference_modes_accepted(self):
    # other parameters and other car modes leave the board in SILENT
    for mode, param in ((TOYOTA, REFERENCE_PARAM + 1), (TOYOTA, REFERENCE_PARAM | 0x100), (HONDA, 0)):
      h = self._set_mode(mode, param)
      self.assertEqual(h["safety_mode"], int(SILENT), (mode, param))
    for mode, param in ((NOOUTPUT, 0), (ELM327, 1), (ELM327, 0)):
      self.assertEqual(self._set_mode(mode, param)["safety_mode"], int(mode))
    h = self._set_mode(TOYOTA, REFERENCE_PARAM)
    self.assertEqual((h["safety_mode"], h["safety_param"]), (int(TOYOTA), REFERENCE_PARAM))

  def test_toyota_leaves_only_to_nooutput_or_silent(self):
    self._set_mode(TOYOTA, REFERENCE_PARAM)
    for mode, param in ((ELM327, 1), (HONDA, 0), (TOYOTA, REFERENCE_PARAM + 1)):
      h = self._set_mode(mode, param)
      self.assertEqual((h["safety_mode"], h["safety_param"]), (int(TOYOTA), REFERENCE_PARAM), (mode, param))
    self.assertEqual(self._set_mode(NOOUTPUT)["safety_mode"], int(NOOUTPUT))
    self._set_mode(TOYOTA, REFERENCE_PARAM)
    self.assertEqual(self._set_mode(SILENT)["safety_mode"], int(SILENT))

  def test_alternative_experience_fixed(self):
    for mode, param in ((SILENT, 0), (NOOUTPUT, 0), (TOYOTA, REFERENCE_PARAM)):
      self._set_mode(mode, param)
      self.p.set_alternative_experience(1)
      time.sleep(SETTLE_S)
      self.assertEqual(self.p.health()["alternative_experience"], 0, mode)

  def test_config_requests_refused_in_car_mode(self):
    p = self.p
    # power save (0xe7): refused in TOYOTA, so camera-side forwarding cannot be stopped with the relay driven
    self._set_mode(TOYOTA, REFERENCE_PARAM)
    p.set_power_save(True)
    time.sleep(SETTLE_S)
    self.assertFalse(p.health()["power_save_enabled"])
    # CAN bit rate (0xde): unchanged in TOYOTA (can_speed is reported in firmware units, so only compare)
    speed = p.can_health(0)["can_speed"]
    p.set_can_speed_kbps(0, OTHER_SPEED)
    time.sleep(SETTLE_S)
    self.assertEqual(p.can_health(0)["can_speed"], speed)

    # the same requests work outside a car mode
    self._set_mode(NOOUTPUT)
    p.set_power_save(True)
    time.sleep(SETTLE_S)
    self.assertTrue(p.health()["power_save_enabled"])
    p.set_power_save(False)
    p.set_can_speed_kbps(0, OTHER_SPEED)
    time.sleep(SETTLE_S)
    self.assertNotEqual(p.can_health(0)["can_speed"], speed)
    p.set_can_speed_kbps(0, SPEED_NORMAL)


if __name__ == "__main__":
  unittest.main()
