"""LionDriver HIL test: per-ID TX rate supervision (TSR-518, design note DN-01).

Verifies SWSR-518 on the device (LionDriver WP-V-05). Needs:
  - a panda flashed with a reference build: LIONDRIVER_REFERENCE=1 scons
  - a panda jungle on the default harness orientation, which plays the car on bus 0
  - HITL=1 REFERENCE_BUILD=1 in the environment

The test engages, sends STEERING_LKA at its nominal 100 Hz (nothing refused), then floods it from the host side and
checks that frames beyond the budget are refused, that authority is revoked and that the health packet reports it.
"""
import os
import time
import unittest

from opendbc.can.packer import CANPacker
from panda.tests.hitl.base import PandaTestCase, panda_test
from panda.tests.hitl.test_12_rx_timeout import DBC, REFERENCE_PARAM, SILENT, TOYOTA, CarSim


@unittest.skipUnless(os.environ.get("REFERENCE_BUILD") == "1", "needs a LIONDRIVER_REFERENCE=1 panda build")
@panda_test(needs_jungle=True)
class TestTxRate(PandaTestCase):
  def setUp(self):
    super().setUp()
    from panda.tests.hitl.base import _panda_jungle
    self.sim = CarSim(_panda_jungle)
    self.packer = CANPacker(DBC)
    self.p.set_safety_mode(TOYOTA, REFERENCE_PARAM)
    self.sim.start()
    self.addCleanup(self.p.set_safety_mode, SILENT)
    self.addCleanup(self.sim.stop)
    time.sleep(0.5)

  def _engage(self):
    self.p.send_heartbeat(engaged=False)
    self.sim.cruise_active = False
    time.sleep(0.1)
    self.p.send_heartbeat(engaged=False)
    self.sim.cruise_active = True
    time.sleep(0.1)
    self.p.send_heartbeat(engaged=True)
    return self.p.health()["controls_allowed"]

  def _lka(self):
    return self.packer.make_can_msg("STEERING_LKA", 0, {"STEER_TORQUE_CMD": 0, "STEER_REQUEST": 0})

  def test_nominal_rate_passes(self):
    self.assertTrue(self._engage())
    before = self.p.health()["safety_tx_blocked"]
    for _ in range(100):  # 1 s at 100 Hz
      self.p.can_send(*self._lka())
      self.p.send_heartbeat(engaged=True)
      time.sleep(0.01)
    health = self.p.health()
    self.assertEqual(health["safety_tx_blocked"], before)
    self.assertTrue(health["controls_allowed"])
    self.assertFalse(health["tx_rate_exceeded"])

  def test_flood_revokes_and_reports(self):
    self.assertTrue(self._engage())
    before = self.p.health()["safety_tx_blocked"]
    self.p.can_send_many([self._lka()] * 100)  # one burst, far above the budget of 36 per 250 ms
    time.sleep(0.05)
    health = self.p.health()
    self.assertGreater(health["safety_tx_blocked"], before)
    self.assertFalse(health["controls_allowed"])
    self.assertTrue(health["tx_rate_exceeded"])

    # cleared by the next safety mode change
    self.p.set_safety_mode(SILENT)
    self.assertFalse(self.p.health()["tx_rate_exceeded"])


if __name__ == "__main__":
  unittest.main()
