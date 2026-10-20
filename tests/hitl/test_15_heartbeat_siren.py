"""LionDriver HIL test: heartbeat loss revokes authority within 0.3 s (TSR-407), a SoC stall revokes it on the
missing command frames (TSR-408), and each sounds the siren (TSR-516(a)/(b), DQ-404 A).

Verifies SWSR-407, SWSR-408 and SWSR-516 (heartbeat loss, SoC stall) on the device (LionDriver WP-V-05). Needs:
  - a panda flashed with a reference build: LIONDRIVER_REFERENCE=1 scons (the car simulation sends the reference
    RX set; the heartbeat monitor and this siren are in every build)
  - a panda jungle on the default harness orientation, which plays the car on bus 0
  - HITL=1 REFERENCE_BUILD=1 in the environment

The heartbeat tests engage, keep sending the actuation frames (so the TSR-408 arrival check does not revoke first),
stop sending the SoC heartbeat and poll the health packet until authority is revoked. The SoC stall test stops the
heartbeat and the frames together, as a frozen SoC does (WP-S-04 T-34): the arrival check revokes first. The
host cannot read the speaker: the siren is confirmed by ear (or a microphone) during the run and recorded in the
bench log; the timing it hangs on (the revocation) is measured here.
"""
import os
import time
import unittest

from panda.tests.hitl.base import PandaTestCase, panda_test
from panda.tests.hitl.test_12_rx_timeout import REFERENCE_PARAM, SILENT, TOYOTA, CarSim, SocSim

TSR407_TIMEOUT_S = 0.3
TSR408_LKA_TIMEOUT_S = 0.05  # STEERING_LKA, the first supervised frame to time out
SAFETY_TASK_PERIOD_S = 0.01
USB_POLL_SLACK_S = 0.05     # health request round trip on the bench
POLL_PERIOD_S = 0.005


@unittest.skipUnless(os.environ.get("REFERENCE_BUILD") == "1", "needs a LIONDRIVER_REFERENCE=1 panda build")
@panda_test(needs_jungle=True)
class TestHeartbeatSiren(PandaTestCase):
  def setUp(self):
    super().setUp()
    from panda.tests.hitl.base import _panda_jungle
    self.sim = CarSim(_panda_jungle)
    self.soc = SocSim(self.p)
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

  def test_revocation_within_tsr407_after_last_heartbeat(self):
    self.assertTrue(self._engage())
    self.soc.step()
    last_heartbeat = time.monotonic()
    elapsed = None
    while time.monotonic() - last_heartbeat < 2.0:
      self.soc.step(heartbeat=False)  # the frames keep arriving, only the heartbeat is lost
      if not self.p.health()["controls_allowed"]:
        elapsed = time.monotonic() - last_heartbeat
        break
    print(f"authority revoked {elapsed} s after the last heartbeat; the siren must sound now (3 s)")
    self.assertIsNotNone(elapsed, "authority not revoked within 2 s of the last heartbeat")
    self.assertGreaterEqual(elapsed, TSR407_TIMEOUT_S - SAFETY_TASK_PERIOD_S, "revoked before the 0.3 s timeout")
    self.assertLessEqual(elapsed, TSR407_TIMEOUT_S + SAFETY_TASK_PERIOD_S + USB_POLL_SLACK_S)

  def test_no_revocation_while_heartbeats_arrive(self):
    self.assertTrue(self._engage())
    end = time.monotonic() + 2.0
    while time.monotonic() < end:
      self.soc.step()
      self.assertTrue(self.p.health()["controls_allowed"])

  def test_soc_stall_revokes_on_missing_frames(self):
    # TSR-408 / TSR-516(b), DQ-404 A: heartbeat and frames stop together; the arrival check revokes first, and the
    # siren must sound now, not 0.3 s later at the heartbeat timeout (which then finds authority already gone)
    self.assertTrue(self._engage())
    for _ in range(20):
      self.soc.step()
    last_frame = time.monotonic()
    elapsed = None
    while time.monotonic() - last_frame < 2.0:
      if not self.p.health()["controls_allowed"]:
        elapsed = time.monotonic() - last_frame
        break
      time.sleep(POLL_PERIOD_S)
    print(f"SoC stall: authority revoked {elapsed} s after the last frame; the siren must sound now (3 s)")
    self.assertIsNotNone(elapsed, "authority not revoked within 2 s of the last frame")
    self.assertLess(elapsed, TSR407_TIMEOUT_S, "revoked by the heartbeat monitor, not the arrival check")
    self.assertLessEqual(elapsed, TSR408_LKA_TIMEOUT_S + SAFETY_TASK_PERIOD_S + USB_POLL_SLACK_S)


if __name__ == "__main__":
  unittest.main()
