"""LionDriver HIL test: lateral actuation refused and reported while the EPS reports a fault (TSR-110).

Verifies SWSR-110 on the device (LionDriver WP-V-05). Needs:
  - a panda flashed with a reference build: LIONDRIVER_REFERENCE=1 scons
  - a panda jungle on the default harness orientation, which plays the car on bus 0
  - HITL=1 REFERENCE_BUILD=1 in the environment

The jungle sends the Toyota RX messages of the reference build, the test engages, then sets EPS_STATUS.LKA_STATE
to a fault code and checks that the health packet reports the fault and that LKA frames with torque are refused.
"""
import os
import time
import unittest

from opendbc.can.packer import CANPacker
from panda.tests.hitl.base import PandaTestCase, panda_test
from panda.tests.hitl.test_12_rx_timeout import DBC, REFERENCE_PARAM, SILENT, TOYOTA, CarSim

FAULT_CODES = (0, 3, 9, 11, 17, 21, 25)
NO_FAULT_CODES = (1, 5)


@unittest.skipUnless(os.environ.get("REFERENCE_BUILD") == "1", "needs a LIONDRIVER_REFERENCE=1 panda build")
@panda_test(needs_jungle=True)
class TestEpsFault(PandaTestCase):
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

  def _lka_tx_blocked(self, torque):
    before = self.p.health()["safety_tx_blocked"]
    addr, dat, bus = self.packer.make_can_msg("STEERING_LKA", 0, {"STEER_TORQUE_CMD": torque, "STEER_REQUEST": 1})
    self.p.can_send(addr, dat, bus)
    time.sleep(0.05)
    self.p.send_heartbeat(engaged=True)
    return self.p.health()["safety_tx_blocked"] > before

  def test_fault_reported_and_lka_refused(self):
    for code in FAULT_CODES + NO_FAULT_CODES:
      with self.subTest(lka_state=code):
        self.sim.lka_state = 5
        time.sleep(0.1)
        self.assertTrue(self._engage())
        self.sim.lka_state = code
        time.sleep(0.1)  # a few EPS_STATUS frames at 25 Hz
        self.p.send_heartbeat(engaged=True)
        health = self.p.health()
        fault = code in FAULT_CODES
        self.assertEqual(fault, health["eps_fault"])
        self.assertTrue(health["controls_allowed"], "an EPS fault must not revoke longitudinal authority")
        self.assertEqual(fault, self._lka_tx_blocked(10))

  def test_flag_cleared_on_mode_change(self):
    self.sim.lka_state = 25
    time.sleep(0.1)
    self.assertTrue(self.p.health()["eps_fault"])
    self.p.set_safety_mode(SILENT)
    self.assertFalse(self.p.health()["eps_fault"])


if __name__ == "__main__":
  unittest.main()
