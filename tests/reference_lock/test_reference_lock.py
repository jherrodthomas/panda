#!/usr/bin/env python3
"""Host tests for board/reference_lock.h (LionDriver TSR-512/513, SWSR-512/512a/513).

The gate exists only in builds with LIONDRIVER_REFERENCE. It is compiled for the host against the real
ControlPacket_t and safety mode numbers. These tests check which host requests the gate lets through;
they do not show the effect of a request on the board (HIL).
"""
import ctypes
import os
import re
import subprocess
import tempfile
import unittest

PANDA_DIR = os.path.realpath(os.path.join(os.path.dirname(__file__), "..", ".."))
OPENDBC_DIR = os.path.realpath(os.path.join(PANDA_DIR, "..", "opendbc_repo"))

HARNESS = r"""
#include <stdbool.h>
#include <stdint.h>

#include "board/comms_definitions.h"
#include "board/health.h"
#include "opendbc/safety/can.h"
#include "opendbc/safety/declarations.h"

uint16_t current_safety_mode = SAFETY_SILENT;
bool is_car_safety_mode(uint16_t mode) {
  return (mode != SAFETY_SILENT) && (mode != SAFETY_NOOUTPUT) && (mode != SAFETY_ALLOUTPUT) && (mode != SAFETY_ELM327);
}
void print(const char *a) { (void)a; }
void puth(unsigned int i) { (void)i; }

#include "board/reference_lock.h"

bool allowed(uint8_t request, uint16_t param1, uint16_t param2) {
  ControlPacket_t req = {.request = request, .param1 = param1, .param2 = param2, .length = 0U};
  return reference_request_allowed(&req);
}
void set_mode(uint16_t mode) { current_safety_mode = mode; }
uint16_t rejected(void) { return reference_requests_rejected; }
void set_rejected(uint16_t n) { reference_requests_rejected = n; }
uint16_t build_flags(void) { return reference_build_health_flags(); }
"""

# A build without LIONDRIVER_REFERENCE has no gate; only the build flag can be checked.
PLAIN_HARNESS = r"""
#include <stdbool.h>
#include <stdint.h>

#include "board/health.h"
#include "board/reference_lock.h"

uint16_t build_flags(void) { return reference_build_health_flags(); }
"""

SILENT, HONDA, TOYOTA, ELM327, ALLOUTPUT, NOOUTPUT = 0, 1, 2, 3, 17, 19
REFERENCE_PARAM = 73
# requests refused in a car mode (SWSR-513); 0xdf is refused in every mode (SWSR-512a)
CAR_MODE_LOCKED = (0xc5, 0xd8, 0xdb, 0xde, 0xe5, 0xe6, 0xe7, 0xe8, 0xf1, 0xf9, 0xfc)
# requests openpilot's pandad sends while onroad; must stay allowed
ALWAYS_ALLOWED = (0xa8, 0xc0, 0xc1, 0xd2, 0xd6, 0xdd, 0xf3, 0xb1, 0xb2)


def build(tmp, name, defines, harness=HARNESS):
  src = os.path.join(tmp, f"{name}.c")
  lib = os.path.join(tmp, f"lib{name}.so")
  with open(src, "w") as f:
    f.write(harness)
  subprocess.check_call(["gcc", "-std=gnu11", "-Wall", "-Wextra", "-Werror", "-shared", "-fPIC", *defines,
                         "-I", PANDA_DIR, "-I", OPENDBC_DIR, "-o", lib, src])
  so = ctypes.CDLL(lib)
  so.build_flags.restype = ctypes.c_uint16
  if harness is not HARNESS:
    return so
  so.allowed.restype = ctypes.c_bool
  so.allowed.argtypes = [ctypes.c_uint8, ctypes.c_uint16, ctypes.c_uint16]
  so.set_mode.argtypes = [ctypes.c_uint16]
  so.rejected.restype = ctypes.c_uint16
  so.set_rejected.argtypes = [ctypes.c_uint16]
  return so


class TestReferenceLock(unittest.TestCase):
  @classmethod
  def setUpClass(cls):
    cls.tmp = tempfile.TemporaryDirectory()
    cls.ref = build(cls.tmp.name, "reference", ["-DLIONDRIVER_REFERENCE"])
    cls.plain = build(cls.tmp.name, "plain", [], PLAIN_HARNESS)

  @classmethod
  def tearDownClass(cls):
    cls.tmp.cleanup()

  def test_safety_mode_requests_outside_toyota(self):
    for current in (SILENT, NOOUTPUT, ELM327):
      self.ref.set_mode(current)
      for mode in (SILENT, NOOUTPUT):
        self.assertTrue(self.ref.allowed(0xdc, mode, 0))
      for param in (0, 1):
        self.assertTrue(self.ref.allowed(0xdc, ELM327, param))
      self.assertTrue(self.ref.allowed(0xdc, TOYOTA, REFERENCE_PARAM))
      # any other parameter (other EPS factor, any flag bit) or mode is refused
      for param in (0, 66, 88, REFERENCE_PARAM | 0x100, REFERENCE_PARAM | 0x400, 0xFFFF):
        self.assertFalse(self.ref.allowed(0xdc, TOYOTA, param), param)
      for mode in (HONDA, ALLOUTPUT, 4, 25):
        self.assertFalse(self.ref.allowed(0xdc, mode, 0), mode)

  def test_safety_mode_requests_in_toyota(self):
    self.ref.set_mode(TOYOTA)
    self.assertTrue(self.ref.allowed(0xdc, NOOUTPUT, 0))
    self.assertTrue(self.ref.allowed(0xdc, SILENT, 0))
    for mode, param in ((TOYOTA, REFERENCE_PARAM), (ELM327, 1), (ALLOUTPUT, 0), (HONDA, 0)):
      self.assertFalse(self.ref.allowed(0xdc, mode, param), (mode, param))

  def test_alternative_experience_fixed(self):
    for mode in (SILENT, NOOUTPUT, ELM327, TOYOTA):
      self.ref.set_mode(mode)
      self.assertFalse(self.ref.allowed(0xdf, 1, 0))

  def test_config_requests_refused_in_car_mode_only(self):
    for request in CAR_MODE_LOCKED:
      self.ref.set_mode(TOYOTA)
      self.assertFalse(self.ref.allowed(request, 1, 1), hex(request))
      for mode in (SILENT, NOOUTPUT, ELM327):
        self.ref.set_mode(mode)
        self.assertTrue(self.ref.allowed(request, 1, 1), hex(request))

  def test_softloader_entry_refused_in_car_mode(self):
    self.ref.set_mode(TOYOTA)
    self.assertFalse(self.ref.allowed(0xd1, 1, 0))
    for mode in (SILENT, NOOUTPUT, ELM327):
      self.ref.set_mode(mode)
      self.assertTrue(self.ref.allowed(0xd1, 1, 0))

  def test_rejections_counted_for_the_health_packet(self):
    self.ref.set_rejected(0)
    self.ref.set_mode(TOYOTA)
    self.assertTrue(self.ref.allowed(0xdc, NOOUTPUT, 0))
    self.assertEqual(self.ref.rejected(), 0, "an accepted request must not be counted")
    self.assertFalse(self.ref.allowed(0xdf, 1, 0))
    self.assertFalse(self.ref.allowed(0xe7, 1, 0))
    self.assertEqual(self.ref.rejected(), 2)
    # saturates instead of wrapping to 0
    self.ref.set_rejected(0xFFFF)
    self.assertFalse(self.ref.allowed(0xdf, 1, 0))
    self.assertEqual(self.ref.rejected(), 0xFFFF)

  def test_reference_build_reported_in_health_flags(self):
    # the host refuses a car safety mode without this flag (TSR-619); it must be set in the reference build only
    reference_build_flag = 1 << 12
    self.assertEqual(self.ref.build_flags(), reference_build_flag)
    self.assertEqual(self.plain.build_flags(), 0)

  def test_reference_build_flag_matches_python_library(self):
    # read the sources rather than import the library (it needs USB and opendbc)
    with open(os.path.join(PANDA_DIR, "python", "__init__.py")) as f:
      lib = f.read()
    self.assertRegex(lib, r"HEALTH_FLAG_REFERENCE_BUILD = 1 << 12\n")
    self.assertIn('"reference_build": bool(flags & self.HEALTH_FLAG_REFERENCE_BUILD)', lib)
    with open(os.path.join(PANDA_DIR, "board", "health.h")) as f:
      flags = re.findall(r"#define (HEALTH_FLAG_\w+)\s+\(\(?(?:uint16_t\))?1U << (\d+)\)", f.read())
    bits = [int(b) for _, b in flags]
    self.assertIn(("HEALTH_FLAG_REFERENCE_BUILD", "12"), flags)
    self.assertEqual(len(bits), len(set(bits)), f"health flag bits must be unique: {flags}")

  def test_pandad_onroad_requests_allowed(self):
    self.ref.set_mode(TOYOTA)
    for request in ALWAYS_ALLOWED:
      self.assertTrue(self.ref.allowed(request, 1, 1), hex(request))


if __name__ == "__main__":
  unittest.main()
