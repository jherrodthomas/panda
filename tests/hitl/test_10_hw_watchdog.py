"""LionDriver HIL test: the hardware watchdog resets the MCU and releases the relay.

Verifies TSR-501 / HWSR-501 on the bench (LionDriver WP-V-05). Needs:
  - a panda flashed with a fault-injection debug build: FAULT_INJECTION=1 scons
  - a panda jungle, so the harness is emulated (the relay is only driven with a harness
    connected) and the relay drive can be observed on the SBU lines
  - HITL=1 FAULT_INJECTION=1 in the environment

For each injected hang the test measures the time from the request until the relay drive
is released, and checks that the panda comes back up in SILENT.
"""
import os
import time
import unittest

from opendbc.car.structs import CarParams
from panda import Panda
from panda.tests.hitl.base import PandaTestCase, panda_test

FAULT_INJECTION_REQUEST = 0xa9
HANG_MAIN_LOOP = 1
HANG_TICK = 2
HANG_IRQ_DISABLED = 3

# Bound for request -> relay released. 175 ms nominal watchdog timeout plus LSI tolerance,
# relay drive release and USB polling. TSR-501 requires 0.1 s (deviation D-1, accepted as
# interim); tighten this when the watchdog is fed from the >= 50 Hz safety task.
RESET_BUDGET_S = float(os.environ.get("WATCHDOG_RESET_BUDGET_S", "0.25"))
# A tick hang starts only at the next 8 Hz tick, up to one tick period after the request;
# that wait is not part of the detection time, so it is added to the budget for that mode.
TICK_PERIOD_S = 0.125
# smallest SBU voltage change counted as a change of relay drive
SBU_DELTA_V = float(os.environ.get("WATCHDOG_SBU_DELTA_V", "0.3"))
POLL_TIMEOUT_S = 2.0


def sbu_voltages(jungle):
  h = jungle.health()
  return {k: v for k, v in h.items() if k.endswith("_voltage") and "_sbu" in k}


def changed_channels(before, after):
  return {k for k in before if abs(after[k] - before[k]) >= SBU_DELTA_V}


@unittest.skipUnless(os.environ.get("FAULT_INJECTION") == "1", "needs a FAULT_INJECTION=1 panda build")
class TestHardwareWatchdog(PandaTestCase):
  def _relay_driven_channels(self):
    p, jungle = self.p, self.panda_jungle
    p.set_safety_mode(CarParams.SafetyModel.silent)
    time.sleep(0.5)
    released = sbu_voltages(jungle)
    p.set_safety_mode(CarParams.SafetyModel.toyota, 73)
    time.sleep(0.5)
    driven = sbu_voltages(jungle)
    channels = changed_channels(released, driven)
    self.assertTrue(channels, f"relay drive not visible on the SBU lines (released {released}, driven {driven})")
    return released, channels

  def _inject_and_time_release(self, mode, start_delay_s=0.0):
    p, jungle = self.p, self.panda_jungle
    released, channels = self._relay_driven_channels()

    t0 = time.monotonic()
    try:
      p._handle.controlWrite(Panda.REQUEST_OUT, FAULT_INJECTION_REQUEST, mode, 0, b'')
    except Exception:
      pass  # the hang may begin before the transfer completes

    elapsed = None
    while time.monotonic() - t0 < POLL_TIMEOUT_S:
      now = sbu_voltages(jungle)
      if not (changed_channels(released, now) & channels):
        elapsed = time.monotonic() - t0
        break
    self.assertIsNotNone(elapsed, f"relay drive not released within {POLL_TIMEOUT_S} s")
    print(f"\nfault injection mode {mode}: relay drive released after {elapsed * 1000:.1f} ms")
    budget = RESET_BUDGET_S + start_delay_s
    self.assertLessEqual(elapsed, budget, f"released after {elapsed:.3f} s, budget {budget:.3f} s")

    # the panda must come back up after the watchdog reset, in SILENT
    p.reconnect()
    health = p.health()
    self.assertEqual(health["safety_mode"], CarParams.SafetyModel.silent)
    self.assertLess(health["uptime"], 5, "panda did not reset")

  @panda_test(needs_jungle=True, timeout=30)
  def test_main_loop_hang(self):
    self._inject_and_time_release(HANG_MAIN_LOOP)

  @panda_test(needs_jungle=True, timeout=30)
  def test_tick_hang(self):
    self._inject_and_time_release(HANG_TICK, start_delay_s=TICK_PERIOD_S)

  @panda_test(needs_jungle=True, timeout=30)
  def test_hang_with_interrupts_disabled(self):
    self._inject_and_time_release(HANG_IRQ_DISABLED)

  @panda_test(needs_jungle=True, timeout=30)
  def test_no_reset_without_fault(self):
    p = self.p
    self._relay_driven_channels()
    up0 = p.health()["uptime"]
    time.sleep(3.0)
    self.assertGreaterEqual(p.health()["uptime"], up0 + 2, "panda reset without an injected fault")
