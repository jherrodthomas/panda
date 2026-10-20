#!/usr/bin/env python3
"""Host tests for board/drivers/heartbeat_siren.h (LionDriver TSR-516(b) / SWSR-516 for heartbeat loss, TSR-407, and
for missing SoC command frames, TSR-408, DQ-404 A).

The siren logic and the heartbeat monitor are compiled for the host together, in the order the 100 Hz safety task
calls them. These tests check when the siren is asked for; the speaker on the device is checked on the HIL bench.
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

#include "board/drivers/heartbeat_monitor.h"
#include "board/drivers/heartbeat_siren.h"

static bool controls_allowed = false;
static bool safety_tx_arrival_revoked = false;
static bool tx_frames_stopped = false;
static uint32_t tx_missing_ticks = 0U;
#define TX_ARRIVAL_TIMEOUT_TICKS 5U  // STEERING_LKA, 50 ms (opendbc toyota.h)

void reset_state(void) {
  heartbeat_age_ticks = 0U; heartbeat_siren_reset(); controls_allowed = false;
  safety_tx_arrival_revoked = false; tx_frames_stopped = false; tx_missing_ticks = 0U;
}
void stop_tx_frames(void) { tx_frames_stopped = true; }
void driver_disengage(void) { controls_allowed = false; }  // brake or cruise off: revoked in the RX hook, no flag
void heartbeat(void) { heartbeat_monitor_reset(); }
void grant(void) { controls_allowed = true; }
bool allowed(void) { return controls_allowed; }
uint32_t hold_ticks(void) { return HEARTBEAT_SIREN_HOLD_TICKS; }
uint32_t timeout_ticks(void) { return HEARTBEAT_AUTHORITY_TIMEOUT_TICKS; }

// the TX-arrival part of opendbc safety_tick() (TSR-408): the flag is set in the tick that revokes granted authority
static void safety_tick(void) {
  tx_missing_ticks = tx_frames_stopped ? (tx_missing_ticks + 1U) : 0U;
  safety_tx_arrival_revoked = controls_allowed && (tx_missing_ticks > TX_ARRIVAL_TIMEOUT_TICKS);
  if (safety_tx_arrival_revoked) {
    controls_allowed = false;
  }
}

// one 100 Hz safety task, in the order of main.c safety_task_handler (car safety mode)
bool task(void) {
  safety_tick();
  const bool heartbeat_revoke = heartbeat_monitor_tick();
  const bool siren = heartbeat_siren_tick(heartbeat_revoke, controls_allowed, safety_tx_arrival_revoked);
  if (heartbeat_revoke) {
    controls_allowed = false;
  }
  return siren;
}
"""

SAFETY_TASK_PERIOD_S = 0.010
TICK_PERIOD_S = 0.125   # 8 Hz tick drives the speaker on every other tick
HOLD_S = 3.0            # as the existing siren on heartbeat loss
SG02_WARNING_S = 1.0    # SG-02 / SG-06: take-over warning within 1 s


class TestHeartbeatSiren(unittest.TestCase):
  @classmethod
  def setUpClass(cls):
    cls.tmp = tempfile.TemporaryDirectory()
    src = os.path.join(cls.tmp.name, "harness.c")
    lib = os.path.join(cls.tmp.name, "libheartbeat_siren.so")
    with open(src, "w") as f:
      f.write(HARNESS)
    subprocess.check_call(["gcc", "-std=gnu11", "-Wall", "-Wextra", "-Werror", "-shared", "-fPIC",
                           "-I", PANDA_DIR, "-o", lib, src])
    cls.lib = ctypes.CDLL(lib)
    for name in ("task", "allowed"):
      getattr(cls.lib, name).restype = ctypes.c_bool
    cls.lib.hold_ticks.restype = ctypes.c_uint32
    cls.lib.timeout_ticks.restype = ctypes.c_uint32

  @classmethod
  def tearDownClass(cls):
    cls.tmp.cleanup()

  def setUp(self):
    self.lib.reset_state()

  def _run(self, n):
    return [self.lib.task() for _ in range(n)]

  def test_hold_is_3s(self):
    self.assertAlmostEqual(self.lib.hold_ticks() * SAFETY_TASK_PERIOD_S, HOLD_S)

  def test_siren_on_revocation_of_granted_authority(self):
    self.lib.grant()
    n = self.lib.timeout_ticks()
    results = self._run(n + 10)
    self.assertFalse(any(results[:n - 1]), "siren before the 0.3 s heartbeat timeout")
    self.assertTrue(results[n - 1], "no siren in the task that revokes authority")
    self.assertFalse(self.lib.allowed(), "authority not revoked")
    self.assertTrue(all(results[n - 1:]))

  def test_siren_within_sg02_budget(self):
    # worst case to sound: revocation task + one task period + two speaker ticks
    first = (self.lib.timeout_ticks() + 1) * SAFETY_TASK_PERIOD_S + 2 * TICK_PERIOD_S
    self.assertLess(first, SG02_WARNING_S)

  def test_no_siren_when_not_engaged(self):
    # heartbeat loss while disengaged revokes nothing the driver relies on
    self.assertFalse(any(self._run(self.lib.timeout_ticks() * 20)))

  def test_no_siren_while_heartbeats_arrive(self):
    self.lib.grant()
    for _ in range(50):
      self.assertFalse(any(self._run(self.lib.timeout_ticks() - 1)))
      self.lib.heartbeat()
    self.assertTrue(self.lib.allowed())

  def test_hold_then_stop(self):
    self.lib.grant()
    n, hold = self.lib.timeout_ticks(), self.lib.hold_ticks()
    self.assertTrue(self._run(n)[-1])
    after = self._run(hold + 5)
    self.assertTrue(all(after[:hold - 1]), "siren shorter than 3 s")
    self.assertFalse(any(after[hold:]), "siren not stopped after 3 s")

  def test_sounds_once_per_revocation(self):
    # the heartbeat stays lost: authority is gone, so the siren is not re-triggered after the hold
    self.lib.grant()
    n, hold = self.lib.timeout_ticks(), self.lib.hold_ticks()
    self._run(n + hold)
    self.assertFalse(any(self._run(hold * 2)))

  def test_retriggers_after_reengagement(self):
    self.lib.grant()
    n, hold = self.lib.timeout_ticks(), self.lib.hold_ticks()
    self._run(n + hold)
    self.lib.heartbeat()
    self.lib.grant()
    results = self._run(n)
    self.assertFalse(any(results[:n - 1]))
    self.assertTrue(results[n - 1], "no siren on the second revocation")


  # TSR-408 / DQ-404 A: a stalled SoC stops its command frames and its heartbeat together; the arrival check revokes
  # first (WP-S-04 T-34) and the siren must sound in that task, not wait for the heartbeat monitor
  def _stall(self):
    self.lib.grant()
    self.lib.stop_tx_frames()
    return self._run(self.lib.timeout_ticks() + 10)

  def test_siren_on_soc_stall_at_arrival_revocation(self):
    results = self._stall()
    first = results.index(True)
    self.assertLess(first, self.lib.timeout_ticks() - 1, "siren waited for the heartbeat monitor")
    self.assertFalse(any(results[:first]))
    self.assertFalse(self.lib.allowed())
    self.assertTrue(all(results[first:]))

  def test_soc_stall_sounds_once(self):
    # the later heartbeat timeout finds authority gone and does not restart the 3 s hold
    results = self._stall()
    first = results.index(True)
    hold = self.lib.hold_ticks()
    after = self._run(hold)
    total = results[first:] + after
    self.assertTrue(all(total[:hold]))
    self.assertFalse(any(total[hold:]), "hold restarted by the heartbeat timeout")

  def test_no_siren_on_driver_disengagement(self):
    self.lib.grant()
    self._run(10)
    self.lib.driver_disengage()
    for _ in range(20):
      self.assertFalse(any(self._run(self.lib.timeout_ticks() - 1)))
      self.lib.heartbeat()
    # the SoC stops after the driver disengaged: nothing granted is revoked
    self.lib.stop_tx_frames()
    self.assertFalse(any(self._run(self.lib.timeout_ticks() * 3)))

  def test_arrival_flag_alone_sounds(self):
    # the arrival check revokes while heartbeats still arrive (a SoC that keeps its heartbeat but stops commanding)
    self.lib.grant()
    self.lib.stop_tx_frames()
    results = []
    for _ in range(10):
      results += self._run(10)
      self.lib.heartbeat()
    self.assertTrue(any(results))
    self.assertFalse(self.lib.allowed())


if __name__ == "__main__":
  unittest.main()
