#!/usr/bin/env python3
"""Host tests for board/drivers/heartbeat_monitor.h (LionDriver TSR-407 / SWSR-407).

The monitor is compiled for the host on its own. These tests check when it asks for authority to be
revoked; the reaction in a car safety mode and the timing on the device are checked on the HIL bench.
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

void reset_state(void) { heartbeat_age_ticks = 0U; }
void reset(void) { heartbeat_monitor_reset(); }
bool tick(void) { return heartbeat_monitor_tick(); }
uint32_t timeout_ticks(void) { return HEARTBEAT_AUTHORITY_TIMEOUT_TICKS; }
"""

SAFETY_TASK_PERIOD_S = 0.010
TSR407_TIMEOUT_S = 0.3
HOST_HEARTBEAT_PERIOD_S = 0.1  # pandad sends the heartbeat at 10 Hz


class TestHeartbeatMonitor(unittest.TestCase):
  @classmethod
  def setUpClass(cls):
    cls.tmp = tempfile.TemporaryDirectory()
    src = os.path.join(cls.tmp.name, "harness.c")
    lib = os.path.join(cls.tmp.name, "libheartbeat_monitor.so")
    with open(src, "w") as f:
      f.write(HARNESS)
    subprocess.check_call(["gcc", "-std=gnu11", "-Wall", "-Wextra", "-Werror", "-shared", "-fPIC",
                           "-I", PANDA_DIR, "-o", lib, src])
    cls.lib = ctypes.CDLL(lib)
    cls.lib.tick.restype = ctypes.c_bool
    cls.lib.timeout_ticks.restype = ctypes.c_uint32

  @classmethod
  def tearDownClass(cls):
    cls.tmp.cleanup()

  def setUp(self):
    self.lib.reset_state()

  def test_timeout_is_tsr407(self):
    self.assertAlmostEqual(self.lib.timeout_ticks() * SAFETY_TASK_PERIOD_S, TSR407_TIMEOUT_S)

  def test_revokes_after_timeout(self):
    n = self.lib.timeout_ticks()
    results = [self.lib.tick() for _ in range(n + 5)]
    self.assertFalse(any(results[:n - 1]), "revoked before 0.3 s")
    self.assertTrue(all(results[n - 1:]), "not revoked after 0.3 s, or revocation not held")

  def test_heartbeat_at_host_rate_never_revokes(self):
    ticks_per_heartbeat = round(HOST_HEARTBEAT_PERIOD_S / SAFETY_TASK_PERIOD_S)
    for _ in range(50):
      self.lib.reset()
      for _ in range(ticks_per_heartbeat):
        self.assertFalse(self.lib.tick())

  def test_two_missed_heartbeats_tolerated(self):
    ticks_per_heartbeat = round(HOST_HEARTBEAT_PERIOD_S / SAFETY_TASK_PERIOD_S)
    self.lib.reset()
    self.assertFalse(any(self.lib.tick() for _ in range(3 * ticks_per_heartbeat - 1)))

  def test_heartbeat_clears_revocation(self):
    for _ in range(self.lib.timeout_ticks() + 1):
      self.lib.tick()
    self.lib.reset()
    self.assertFalse(self.lib.tick())


if __name__ == "__main__":
  unittest.main()
