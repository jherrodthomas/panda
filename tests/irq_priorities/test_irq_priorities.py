#!/usr/bin/env python3
"""Static check for board/drivers/irq_priorities.h (LionDriver TSR-519 / SWSR-519, WP-W-03 §5.4.4).

Every interrupt the panda firmware enables must get an explicit priority in interrupt_priorities_init(),
and the classes must stay equal until the priority scheme of WP-W-03 §5.4.2 is adopted. These are source
checks; the priorities actually set on the device are checked on the HIL bench.
"""
import glob
import os
import re
import unittest

PANDA_DIR = os.path.realpath(os.path.join(os.path.dirname(__file__), "..", ".."))
BOARD_DIR = os.path.join(PANDA_DIR, "board")
PRIORITIES_H = os.path.join(BOARD_DIR, "drivers", "irq_priorities.h")

# The body firmware and the vendor headers are not part of the panda image.
EXCLUDED_DIRS = (os.path.join(BOARD_DIR, "body"), os.path.join(BOARD_DIR, "stm32h7", "inc"))


def _sources():
  for fn in glob.glob(os.path.join(BOARD_DIR, "**", "*.[ch]"), recursive=True):
    if not fn.startswith(EXCLUDED_DIRS):
      yield fn


def _strip_comments(src):
  src = re.sub(r"/\*.*?\*/", "", src, flags=re.S)
  return re.sub(r"//[^\n]*", "", src)


class TestIrqPriorities(unittest.TestCase):
  @classmethod
  def setUpClass(cls):
    with open(PRIORITIES_H) as f:
      cls.priorities_src = _strip_comments(f.read())
    cls.prioritised = dict(re.findall(r"NVIC_SetPriority\(\s*(\w+)\s*,\s*(\w+)\s*\)", cls.priorities_src))
    cls.enabled = set()
    for fn in _sources():
      with open(fn) as f:
        cls.enabled.update(re.findall(r"NVIC_EnableIRQ\(\s*(\w+)\s*\)", _strip_comments(f.read())))

  def test_enable_sites_found(self):
    # Guard against the scan silently matching nothing.
    self.assertIn("SAFETY_TASK_TIMER_IRQ", self.enabled)
    self.assertIn("FDCAN1_IT0_IRQn", self.enabled)

  def test_every_enabled_irq_has_a_priority(self):
    missing = sorted(self.enabled - set(self.prioritised))
    self.assertEqual(missing, [], f"IRQs enabled without an explicit priority in irq_priorities.h: {missing}")

  def test_no_priority_for_unused_irq(self):
    unused = sorted(set(self.prioritised) - self.enabled)
    self.assertEqual(unused, [], f"irq_priorities.h sets a priority for IRQs that are never enabled: {unused}")

  def test_priorities_use_class_macros(self):
    for irq, prio in self.prioritised.items():
      self.assertRegex(prio, r"^IRQ_PRIO_\w+$", f"{irq} uses a literal priority")

  def test_classes_equal_until_scheme_adopted(self):
    # WP-W-03 §6.3 relies on equal priorities (no preemption between ISRs). Changing this test is part of
    # adopting the scheme of §5.4.2, after the §5.4.5 timing measurements.
    levels = dict(re.findall(r"#define\s+(IRQ_PRIO_\w+)\s+(\d+)U", self.priorities_src))
    self.assertEqual(set(levels), set(self.prioritised.values()))
    self.assertEqual(len(set(levels.values())), 1, f"priority classes differ: {levels}")
    self.assertLess(int(next(iter(levels.values()))), 16)

  def test_init_called_before_irqs_enabled(self):
    with open(os.path.join(BOARD_DIR, "main.c")) as f:
      main_src = _strip_comments(f.read())
    body = main_src[main_src.index("int main(void)"):]
    self.assertLess(body.index("interrupt_priorities_init();"), body.index("clock_init();"))


if __name__ == "__main__":
  unittest.main()
