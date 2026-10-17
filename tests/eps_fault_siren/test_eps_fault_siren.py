#!/usr/bin/env python3
"""Host tests for board/drivers/eps_fault_siren.h (LionDriver TSR-516(b) / SWSR-516 for the EPS fault, TSR-110).

The logic is compiled for the host on its own. These tests check when it asks for the siren; the speaker on the
device is checked on the HIL bench.
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

#include "board/drivers/eps_fault_siren.h"

void reset_state(void) { eps_fault_siren_reset(); }
bool tick(bool eps_fault, bool engaged) { return eps_fault_siren_tick(eps_fault, engaged); }
uint32_t persist_ticks(void) { return EPS_FAULT_SIREN_PERSIST_TICKS; }
uint32_t hold_ticks(void) { return EPS_FAULT_SIREN_HOLD_TICKS; }
"""

SAFETY_TASK_PERIOD_S = 0.010
PERSIST_S = 1.0  # decision 2026-10-12: siren only for faults that persist 1 s
HOLD_S = 3.0     # as the siren on heartbeat loss


class TestEpsFaultSiren(unittest.TestCase):
  @classmethod
  def setUpClass(cls):
    cls.tmp = tempfile.TemporaryDirectory()
    src = os.path.join(cls.tmp.name, "harness.c")
    lib = os.path.join(cls.tmp.name, "libeps_fault_siren.so")
    with open(src, "w") as f:
      f.write(HARNESS)
    subprocess.check_call(["gcc", "-std=gnu11", "-Wall", "-Wextra", "-Werror", "-shared", "-fPIC",
                           "-I", PANDA_DIR, "-o", lib, src])
    cls.lib = ctypes.CDLL(lib)
    cls.lib.tick.restype = ctypes.c_bool
    cls.lib.tick.argtypes = [ctypes.c_bool, ctypes.c_bool]
    cls.lib.persist_ticks.restype = ctypes.c_uint32
    cls.lib.hold_ticks.restype = ctypes.c_uint32

  @classmethod
  def tearDownClass(cls):
    cls.tmp.cleanup()

  def setUp(self):
    self.lib.reset_state()

  def _run(self, n, eps_fault, engaged):
    return [self.lib.tick(eps_fault, engaged) for _ in range(n)]

  def test_constants(self):
    self.assertAlmostEqual(self.lib.persist_ticks() * SAFETY_TASK_PERIOD_S, PERSIST_S)
    self.assertAlmostEqual(self.lib.hold_ticks() * SAFETY_TASK_PERIOD_S, HOLD_S)

  def test_siren_after_persistent_fault(self):
    n = self.lib.persist_ticks()
    results = self._run(n + 10, True, True)
    self.assertFalse(any(results[:n - 1]), "siren before the fault persisted 1 s")
    self.assertTrue(all(results[n - 1:]), "no siren after 1 s, or siren not held while the fault persists")

  def test_no_siren_for_short_faults(self):
    # repeated faults shorter than 1 s (message drop-outs) never sound the siren
    for _ in range(20):
      self.assertFalse(any(self._run(self.lib.persist_ticks() - 1, True, True)))
      self.assertFalse(any(self._run(5, False, True)))

  def test_no_siren_when_not_engaged(self):
    self.assertFalse(any(self._run(self.lib.persist_ticks() * 5, True, False)))

  def test_hold_after_fault_clears(self):
    n, hold = self.lib.persist_ticks(), self.lib.hold_ticks()
    self.assertTrue(self._run(n, True, True)[-1])
    after = self._run(hold + 5, False, True)
    self.assertTrue(all(after[:hold - 1]), "siren shorter than 3 s after the fault cleared")
    self.assertFalse(any(after[hold:]), "siren not stopped 3 s after the fault cleared")

  def test_hold_after_disengagement(self):
    # the driver disengaging (brake) after the siren started does not cut the siren short
    n, hold = self.lib.persist_ticks(), self.lib.hold_ticks()
    self.assertTrue(self._run(n, True, True)[-1])
    self.assertTrue(all(self._run(hold - 1, True, False)))

  def test_siren_after_revocation_during_fault(self):
    # DQ-367: a fault seen while authority was granted still sounds after 1 s when authority is revoked meanwhile
    # (SWSR-408: the host's refused torque frames revoke it after 50 ms)
    n = self.lib.persist_ticks()
    first = self._run(5, True, True)
    rest = self._run(n + 10, True, False)
    results = first + rest
    self.assertFalse(any(results[:n - 1]), "siren before the fault persisted 1 s")
    self.assertTrue(all(results[n - 1:]), "no siren 1 s after a fault that began while engaged")

  def test_no_siren_after_fault_cleared_without_authority(self):
    # once the fault clears, a new fault without authority does not inherit the earlier engagement
    n = self.lib.persist_ticks()
    self._run(5, True, True)
    self._run(1, False, False)
    self.assertFalse(any(self._run(n * 2, True, False)))

  def test_persistence_restarts_after_clear(self):
    n = self.lib.persist_ticks()
    self._run(n - 1, True, True)
    self._run(1, False, True)
    self.assertFalse(any(self._run(n - 1, True, True)))


if __name__ == "__main__":
  unittest.main()
