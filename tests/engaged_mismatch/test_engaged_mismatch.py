#!/usr/bin/env python3
"""Host tests for board/drivers/engaged_mismatch.h (LionDriver TSR-307(a), TSR-516(b)/(c)).

The mismatch logic and the heartbeat-loss siren it starts are compiled for the host together, in the order the 100 Hz
safety task calls them (reference build). The car side is modelled as the envelope sees it: PCM cruise active or not,
and authority revoked when cruise drops (TSR-301). The speaker on the device is checked on the HIL bench.
"""
import ctypes
import os
import subprocess
import tempfile
import unittest

PANDA_DIR = os.path.realpath(os.path.join(os.path.dirname(__file__), "..", ".."))

HARNESS = r"""
#include <stdbool.h>
#include <stdint.h>

#include "board/drivers/heartbeat_siren.h"
#include "board/drivers/engaged_mismatch.h"

static bool controls_allowed = false;
static bool heartbeat_engaged = false;
static bool cruise_engaged_prev = false;

void reset_state(void) {
  engaged_mismatch_reset(); heartbeat_siren_reset();
  controls_allowed = false; heartbeat_engaged = false; cruise_engaged_prev = false;
}
void engage(void) { cruise_engaged_prev = true; heartbeat_engaged = true; controls_allowed = true; }
void set_heartbeat_engaged(bool e) { heartbeat_engaged = e; }
// the envelope revokes on the PCM cruise drop (TSR-301)
void cruise_drop(void) { cruise_engaged_prev = false; controls_allowed = false; }
// PCM cruise state alone, without the envelope's revocation (a mode in which a cruise drop does not revoke)
void set_cruise(bool c) { cruise_engaged_prev = c; }
bool allowed(void) { return controls_allowed; }
uint32_t revoke_ticks(void) { return ENGAGED_MISMATCH_REVOKE_TICKS; }
uint32_t siren_ticks(void) { return ENGAGED_MISMATCH_SIREN_TICKS; }
uint32_t hold_ticks(void) { return HEARTBEAT_SIREN_HOLD_TICKS; }

// one 100 Hz safety task, in the order of main.c safety_task_handler (reference build, heartbeats arriving)
static bool last_start = false;
bool task(void) {
  bool siren = false;
  const bool revoke = engaged_mismatch_tick(controls_allowed, heartbeat_engaged, cruise_engaged_prev, &siren);
  last_start = siren;
  const bool sounding = heartbeat_siren_tick(false, controls_allowed, siren);
  if (revoke) {
    controls_allowed = false;
  }
  return sounding;
}
bool siren_started(void) { return last_start; }
"""

SAFETY_TASK_PERIOD_S = 0.010


class TestEngagedMismatch(unittest.TestCase):
  @classmethod
  def setUpClass(cls):
    cls.tmp = tempfile.TemporaryDirectory()
    src = os.path.join(cls.tmp.name, "harness.c")
    lib = os.path.join(cls.tmp.name, "libengaged_mismatch.so")
    with open(src, "w") as f:
      f.write(HARNESS)
    subprocess.check_call(["gcc", "-std=gnu11", "-Wall", "-Wextra", "-Werror", "-shared", "-fPIC",
                           "-I", PANDA_DIR, "-o", lib, src])
    cls.lib = ctypes.CDLL(lib)
    for name in ("task", "allowed", "siren_started"):
      getattr(cls.lib, name).restype = ctypes.c_bool
    for name in ("revoke_ticks", "siren_ticks", "hold_ticks"):
      getattr(cls.lib, name).restype = ctypes.c_uint32
    cls.lib.set_heartbeat_engaged.argtypes = [ctypes.c_bool]
    cls.lib.set_cruise.argtypes = [ctypes.c_bool]

  @classmethod
  def tearDownClass(cls):
    cls.tmp.cleanup()

  def setUp(self):
    self.lib.reset_state()

  def _run(self, n):
    out = []
    for _ in range(n):
      sounding = self.lib.task()
      out.append((self.lib.allowed(), self.lib.siren_started(), sounding))
    return out

  def _not_engaged(self):
    self.lib.engage()
    self._run(10)
    self.lib.set_heartbeat_engaged(False)

  def test_timing_constants(self):
    self.assertAlmostEqual(self.lib.revoke_ticks() * SAFETY_TASK_PERIOD_S, 0.3)  # TSR-307(a)
    self.assertAlmostEqual(self.lib.siren_ticks() * SAFETY_TASK_PERIOD_S, 0.5)   # TSR-516(c)

  def test_stall_revokes_at_0_3_s_and_sounds_at_0_5_s(self):
    # selfdrived stalls, pandad keeps the heartbeat with "not engaged", no cancel reaches the PCM: cruise stays on
    self._not_engaged()
    r = self._run(self.lib.siren_ticks() + 10)
    n, s = self.lib.revoke_ticks(), self.lib.siren_ticks()
    self.assertTrue(all(a for a, _, _ in r[:n - 1]), "revoked before 0.3 s")
    self.assertFalse(r[n - 1][0], "not revoked at 0.3 s")
    starts = [i for i, (_, st, _) in enumerate(r) if st]
    self.assertEqual(starts, [s - 1], "siren not started exactly once at 0.5 s")
    self.assertFalse(any(snd for _, _, snd in r[:s - 1]))
    self.assertTrue(all(snd for _, _, snd in r[s - 1:]))

  def test_siren_once_while_cruise_stays_on(self):
    self._not_engaged()
    r = self._run(self.lib.siren_ticks() + self.lib.hold_ticks() + 200)
    self.assertEqual(sum(st for _, st, _ in r), 1)
    self.assertFalse(r[-1][2], "siren not stopped after its 3 s hold")

  def test_fast_cancel_silent(self):
    # openpilot leaves engagement and its cancel drops PCM cruise at 0.1 s: the envelope revokes, no siren
    self._not_engaged()
    r = self._run(10)
    self.lib.cruise_drop()
    r += self._run(self.lib.hold_ticks())
    self.assertFalse(any(st or snd for _, st, snd in r))

  def test_slow_cancel_inside_0_5_s_revoked_silently(self):
    # the cancel takes 0.4 s: the mismatch revokes at 0.3 s, cruise drops at 0.4 s, no siren
    self._not_engaged()
    r = self._run(40)
    self.assertFalse(r[-1][0], "not revoked at 0.3 s")
    self.lib.cruise_drop()
    r += self._run(self.lib.hold_ticks())
    self.assertFalse(any(st or snd for _, st, snd in r))

  def test_no_siren_when_cruise_dropped_without_revocation(self):
    # TSR-516(c) asks for the siren only while PCM cruise is still active, whatever revoked the authority
    self._not_engaged()
    self._run(20)
    self.lib.set_cruise(False)
    r = self._run(self.lib.hold_ticks())
    self.assertFalse(any(st or snd for _, st, snd in r))

  def test_cruise_resumed_after_cancel_does_not_sound(self):
    # the cancel takes effect at 0.4 s and the driver resumes the stock cruise at 0.45 s with openpilot still not
    # engaged: the episode ended with the cruise drop, so there is no siren at 0.5 s
    self._not_engaged()
    self._run(40)
    self.lib.cruise_drop()
    self._run(5)
    self.lib.set_cruise(True)
    r = self._run(self.lib.hold_ticks())
    self.assertFalse(any(st or snd for _, st, snd in r))

  def test_heartbeat_engaged_again_before_0_3_s(self):
    self._not_engaged()
    self._run(self.lib.revoke_ticks() - 5)
    self.lib.set_heartbeat_engaged(True)
    r = self._run(self.lib.siren_ticks() * 2)
    self.assertTrue(all(a and not st and not snd for a, st, snd in r))

  def test_no_episode_without_authority(self):
    # heartbeat "not engaged" with authority not granted (disengaged driving) does nothing
    r = self._run(self.lib.siren_ticks() * 4)
    self.assertFalse(any(a or st or snd for a, st, snd in r))

  def test_reset_clears_a_pending_episode(self):
    self._not_engaged()
    self._run(self.lib.revoke_ticks() - 1)
    self.lib.reset_state()
    self.lib.engage()
    self.lib.set_heartbeat_engaged(False)
    r = self._run(self.lib.revoke_ticks() - 1)
    self.assertTrue(all(a for a, _, _ in r), "the episode survived the reset")


if __name__ == "__main__":
  unittest.main()
