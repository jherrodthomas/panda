"""LionDriver HIL test: heartbeat "not engaged" while authority is granted revokes within 0.3 s (TSR-307(a)) and
sounds the siren at 0.5 s only when PCM cruise stays active (TSR-516(c)).

Verifies SWSR-307 and SWSR-516 (engaged mismatch) on the device (LionDriver WP-V-05 VS-FI-10). Needs:
  - a panda flashed with a reference build: LIONDRIVER_REFERENCE=1 scons (engaged_mismatch.h is reference-only)
  - a panda jungle on the default harness orientation, which plays the car on bus 0
  - HITL=1 REFERENCE_BUILD=1 in the environment

Each test engages, then the host plays a SoC whose selfdrived has stopped while pandad keeps the heartbeat: the
heartbeat reports "not engaged" and the command frames keep arriving (so TSR-407 and TSR-408 do not revoke). The car
simulation decides whether the cruise cancel "took effect":
  - cruise stays active: authority goes within 0.3 s and the siren must start 0.5 s after the first such heartbeat
  - cruise drops before the heartbeat changes (an ordinary disengagement): the envelope revokes and the siren stays
    silent
  - cruise drops after the revocation but before 0.5 s (the cancel took effect): the siren stays silent

The host cannot read the speaker. The revocation timing is measured here; each test prints a line stamped with the
time since the first "not engaged" heartbeat saying whether the siren must sound, and the operator confirms it by ear
(or a microphone) and records it in the bench log.
"""
import os
import time
import unittest

from panda.tests.hitl.base import PandaTestCase, panda_test
from panda.tests.hitl.test_12_rx_timeout import REFERENCE_PARAM, SILENT, TOYOTA, CarSim, SocSim

TSR307A_MISMATCH_S = 0.3    # ENGAGED_MISMATCH_REVOKE_TICKS at 100 Hz
TSR516C_SIREN_S = 0.5       # ENGAGED_MISMATCH_SIREN_TICKS at 100 Hz
SAFETY_TASK_PERIOD_S = 0.01
USB_POLL_SLACK_S = 0.05     # health request round trip on the bench
PCM_CRUISE_PERIOD_S = 1 / 33
CANCEL_AT_S = 0.4           # cruise drop after the revocation; the firmware sees it before 0.5 s
OBSERVE_S = 1.5             # keep the mismatch going past the siren time


@unittest.skipUnless(os.environ.get("REFERENCE_BUILD") == "1", "needs a LIONDRIVER_REFERENCE=1 panda build")
@panda_test(needs_jungle=True)
class TestEngagedMismatchSiren(PandaTestCase):
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
    ok = self.p.health()["controls_allowed"]
    for _ in range(20):
      self.soc.step()
    return ok

  def _mismatch(self, cancel_at=None):
    """Sends "not engaged" heartbeats with the frames for OBSERVE_S, drops PCM cruise at cancel_at (s) if given, and
    returns the time from the first such heartbeat to the revocation (None if authority stayed)."""
    first = time.monotonic()
    revoked = None
    while (t := time.monotonic() - first) < OBSERVE_S:
      if cancel_at is not None and t >= cancel_at:
        self.sim.cruise_active = False
      self.soc.step(engaged=False)
      allowed = self.p.health()["controls_allowed"]
      if revoked is None and not allowed:
        revoked = time.monotonic() - first
      elif revoked is not None:
        self.assertFalse(allowed, f"authority granted again {t:.3f} s into the mismatch")
    return revoked

  def test_cruise_stays_active_revokes_and_sounds(self):
    self.assertTrue(self._engage())
    print("t=0.000 s: heartbeat 'not engaged', PCM cruise stays active")
    revoked = self._mismatch()
    print(f"t={revoked} s: authority revoked; the siren MUST start at t={TSR516C_SIREN_S} s (3 s)")
    self.assertIsNotNone(revoked, f"authority not revoked within {OBSERVE_S} s")
    # the episode counts from the first task that sees the heartbeat, so one task period early at most
    self.assertGreaterEqual(revoked, TSR307A_MISMATCH_S - 2 * SAFETY_TASK_PERIOD_S, "revoked before 0.3 s")
    self.assertLessEqual(revoked, TSR307A_MISMATCH_S + SAFETY_TASK_PERIOD_S + USB_POLL_SLACK_S)

  def test_cruise_drops_first_stays_silent(self):
    # an ordinary disengagement: the PCM drops cruise, the envelope revokes (TSR-302), then the heartbeat follows.
    # The episode needs granted authority to start, so none starts
    self.assertTrue(self._engage())
    self.sim.cruise_active = False
    t0 = time.monotonic()
    while self.p.health()["controls_allowed"]:
      self.soc.step()
      self.assertLess(time.monotonic() - t0, 0.5, "PCM cruise drop did not revoke authority")
    print("t=0.000 s: PCM cruise dropped and authority revoked, then heartbeat 'not engaged'")
    self.assertIsNotNone(self._mismatch())  # revoked from the first poll and never granted again
    print(f"t={OBSERVE_S} s: the siren MUST have stayed silent")

  def test_cancel_after_revoke_stays_silent(self):
    # the cancel took effect after the revocation but before the siren time: the episode ends silently
    self.assertTrue(self._engage())
    print(f"t=0.000 s: heartbeat 'not engaged', PCM cruise drops at t={CANCEL_AT_S} s")
    revoked = self._mismatch(cancel_at=CANCEL_AT_S)
    print(f"t={revoked} s: authority revoked; t={OBSERVE_S} s: the siren MUST have stayed silent")
    self.assertIsNotNone(revoked)
    self.assertLessEqual(revoked, TSR307A_MISMATCH_S + SAFETY_TASK_PERIOD_S + USB_POLL_SLACK_S)
    self.assertLess(CANCEL_AT_S + PCM_CRUISE_PERIOD_S + SAFETY_TASK_PERIOD_S, TSR516C_SIREN_S)


if __name__ == "__main__":
  unittest.main()
