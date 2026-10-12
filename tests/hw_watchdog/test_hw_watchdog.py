#!/usr/bin/env python3
"""Host tests for board/drivers/hw_watchdog.h (LionDriver, TSR-502).

The driver is compiled for the host against a fake IWDG register block, so these tests
check its logic: when the watchdog starts, how it is configured, and when it is fed.
They cannot show that the IWDG resets the MCU; that needs the HIL fault-injection test.
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
#include <string.h>

typedef struct {
  volatile uint32_t KR;
  volatile uint32_t PR;
  volatile uint32_t RLR;
  volatile uint32_t SR;
  volatile uint32_t WINR;
} IWDG_TypeDef;

IWDG_TypeDef fake_iwdg;
#define IND_WDG (&fake_iwdg)
#define IWDG_SR_PVU (1UL << 0)
#define IWDG_SR_RVU (1UL << 1)
#define IWDG_SR_WVU (1UL << 2)

#include "board/drivers/hw_watchdog.h"

void reset_state(void) {
  hw_watchdog_running = false;
  hw_watchdog_main_loop_ran = false;
  hw_watchdog_skip_tick = false;
  memset(&fake_iwdg, 0, sizeof(fake_iwdg));
}
void start(void) { hw_watchdog_start(); }
void main_loop_alive(void) { hw_watchdog_main_loop_alive(); }
void tick(void) { hw_watchdog_tick(); }
bool running(void) { return hw_watchdog_running; }
uint32_t prescaler(void) { return HW_WATCHDOG_PRESCALER; }
uint32_t reload(void) { return HW_WATCHDOG_RELOAD; }
uint32_t window(void) { return HW_WATCHDOG_WINDOW; }
"""

FEED_KEY = 0xAAAA
FEED_PERIOD_S = 1.0 / 100.0  # fed from the 100 Hz safety task
# TSR-501 requires a reset within 0.1 s of a stall (SG-07 forwarding FDTI, WP-S-04 §3.6).
TSR501_DETECTION_BUDGET_S = 0.100
LSI_HZ = 32000.0
# assumed LSI tolerance until confirmed from the STM32H725 datasheet (impact record OI-2)
LSI_TOLERANCE = 0.10


class IWDG(ctypes.Structure):
  _fields_ = [(name, ctypes.c_uint32) for name in ("KR", "PR", "RLR", "SR", "WINR")]


class TestHwWatchdog(unittest.TestCase):
  @classmethod
  def setUpClass(cls):
    cls.tmp = tempfile.TemporaryDirectory()
    src = os.path.join(cls.tmp.name, "harness.c")
    lib = os.path.join(cls.tmp.name, "libhw_watchdog.so")
    with open(src, "w") as f:
      f.write(HARNESS)
    subprocess.check_call(["gcc", "-std=gnu11", "-Wall", "-Wextra", "-Werror", "-shared", "-fPIC",
                           "-I", PANDA_DIR, "-o", lib, src])
    cls.lib = ctypes.CDLL(lib)
    cls.lib.running.restype = ctypes.c_bool
    cls.lib.prescaler.restype = ctypes.c_uint32
    cls.lib.reload.restype = ctypes.c_uint32
    cls.lib.window.restype = ctypes.c_uint32
    cls.iwdg = IWDG.in_dll(cls.lib, "fake_iwdg")

  @classmethod
  def tearDownClass(cls):
    cls.tmp.cleanup()

  def setUp(self):
    self.lib.reset_state()

  def test_not_started_before_relay_mode(self):
    self.assertFalse(self.lib.running())
    self.lib.main_loop_alive()
    self.lib.tick()
    self.assertEqual(self.iwdg.KR, 0, "a stopped watchdog must not be touched")

  def test_start_configures_window_mode(self):
    self.lib.start()
    self.assertTrue(self.lib.running())
    self.assertEqual(self.iwdg.PR, self.lib.prescaler())
    self.assertEqual(self.iwdg.RLR, self.lib.reload())
    # the window is written last: that enables window mode and reloads the counter
    self.assertEqual(self.iwdg.WINR, self.lib.window())
    self.assertLess(self.lib.window(), self.lib.reload())

  def test_first_tick_after_start_skipped(self):
    self.lib.start()
    self.iwdg.KR = 0
    self.lib.main_loop_alive()
    self.lib.tick()
    self.assertEqual(self.iwdg.KR, 0, "first tick after start may fall in the closed window")
    self.lib.tick()
    self.assertEqual(self.iwdg.KR, FEED_KEY, "main-loop progress must be kept across the skipped tick")

  def test_start_is_idempotent(self):
    self.lib.start()
    self.iwdg.KR = 0
    self.lib.start()
    self.assertEqual(self.iwdg.KR, 0, "a second start must not write the IWDG again")

  def test_fed_only_when_main_loop_ran(self):
    self.lib.start()
    self.lib.tick()  # skipped first tick
    self.iwdg.KR = 0
    self.lib.main_loop_alive()
    self.lib.tick()
    self.assertEqual(self.iwdg.KR, FEED_KEY)

    # main loop stalled: the next tick must not feed
    self.iwdg.KR = 0
    self.lib.tick()
    self.assertEqual(self.iwdg.KR, 0, "watchdog fed although the main loop did not run")

  def test_stuck_update_flags_do_not_hang(self):
    self.iwdg.SR = 0x7
    self.lib.start()  # must return although PVU/RVU/WVU never clear
    self.assertTrue(self.lib.running())

  def test_timeout_fits_feed_period_and_tsr501_budget(self):
    timeout_s = 4 * (2 ** self.lib.prescaler()) * (self.lib.reload() + 1) / LSI_HZ
    self.assertGreater(timeout_s, 5 * FEED_PERIOD_S, "timeout too close to the feed period")
    self.assertLess(timeout_s, TSR501_DETECTION_BUDGET_S, "timeout longer than the TSR-501 detection budget")

  def test_window_opens_well_before_the_next_feed(self):
    # time after a feed before the next feed is accepted: longest with the slowest LSI
    open_after_s = 4 * (2 ** self.lib.prescaler()) * (self.lib.reload() - self.lib.window()) / (LSI_HZ * (1 - LSI_TOLERANCE))
    self.assertLess(open_after_s, 0.6 * FEED_PERIOD_S, "window still closed when the next feed is due")
    # shortest with the fastest LSI; a safety task running much too fast must still be caught
    open_after_fast_s = 4 * (2 ** self.lib.prescaler()) * (self.lib.reload() - self.lib.window()) / (LSI_HZ * (1 + LSI_TOLERANCE))
    self.assertGreater(open_after_fast_s, 0.3 * FEED_PERIOD_S, "window too short to catch a safety task running too fast")


if __name__ == "__main__":
  unittest.main()
