#!/usr/bin/env python3
"""Host test for board/main_comms.h (LionDriver SWSR-513c): every control request that uses a host-supplied
parameter as an array index checks it against the array bound first, in every build.

comms_control_handler needs most of the board to compile, so this test reads the source: within each `case`
of the handler, every `[req->paramN]` index must come after a `req->paramN <` comparison in the same case.
A new handler without a bound fails here.
"""
import os
import re
import unittest

PANDA_DIR = os.path.realpath(os.path.join(os.path.dirname(__file__), "..", ".."))
MAIN_COMMS = os.path.join(PANDA_DIR, "board", "main_comms.h")

INDEX = re.compile(r"\[\s*req->(param[12])\s*\]")


def handler_cases(src: str) -> dict[str, str]:
  body = src[src.index("int comms_control_handler("):]
  parts = re.split(r"\n\s*case (0x[0-9a-fA-F]+):", body)
  return {parts[i].lower(): parts[i + 1] for i in range(1, len(parts) - 1, 2)}


def unguarded(case_body: str) -> list[str]:
  out = []
  for m in INDEX.finditer(case_body):
    before = case_body[:m.start()]
    if not re.search(r"req->" + m.group(1) + r"\s*<", before):
      out.append(m.group(0))
  return out


class TestCommsBounds(unittest.TestCase):
  def setUp(self):
    with open(MAIN_COMMS, encoding="utf-8") as f:
      self.cases = handler_cases(f.read())

  def test_handler_found(self):
    for code in ("0xc2", "0xc4", "0xde", "0xe8", "0xf1", "0xf9", "0xfc"):
      self.assertIn(code, self.cases)

  def test_every_index_is_bounded(self):
    bad = {code: unguarded(body) for code, body in self.cases.items() if unguarded(body)}
    self.assertEqual(bad, {}, "host-supplied index used without a bound check")

  def test_detects_a_missing_bound(self):
    self.assertEqual(unguarded("  bus_config[req->param1].canfd_auto = 1U;\n"), ["[req->param1]"])
    self.assertEqual(unguarded("  if (req->param1 < PANDA_CAN_CNT) { bus_config[req->param1].x = 1U; }\n"), [])


if __name__ == "__main__":
  unittest.main()
