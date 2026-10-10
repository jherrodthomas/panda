"""Unit tests for board/sys/safety_supervisor.h (LionDriver TSC LD-TSC-001, G1 layer 1).

The supervisor is pure C with no register access; this builds it for the host and drives it
through ctypes. The STM32H7 glue (watchdog, RAM ECC) needs hardware-in-the-loop testing.
"""
import ctypes
import os
import random
import re
import subprocess
import tempfile
import unittest
import zlib

HERE = os.path.dirname(os.path.abspath(__file__))
PANDA = os.path.abspath(os.path.join(HERE, "..", ".."))


def fault_bits():
  with open(os.path.join(PANDA, "board", "sys", "sys.h")) as f:
    return {m.group(1): 1 << int(m.group(2)) for m in re.finditer(r"#define (FAULT_\w+)\s+\(1UL << (\d+)\)", f.read())}


FAULTS = fault_bits()


class Redundant(ctypes.Structure):
  _fields_ = [("value", ctypes.c_uint32), ("inverse", ctypes.c_uint32)]


class Supervisor(ctypes.Structure):
  _fields_ = [("latched_faults", Redundant), ("checkpoints", ctypes.c_uint32), ("ticks_since_housekeeping", ctypes.c_uint32)]


class CodeCrc(ctypes.Structure):
  _fields_ = [("start", ctypes.c_void_p), ("len", ctypes.c_uint32), ("offset", ctypes.c_uint32), ("crc", ctypes.c_uint32),
              ("reference_set", ctypes.c_bool), ("reference", Redundant)]


CP_MAIN_LOOP, CP_HOUSEKEEPING = 1, 2
CODE_CRC_BUSY, CODE_CRC_OK, CODE_CRC_MISMATCH = 0, 1, 2


def build():
  out = os.path.join(tempfile.mkdtemp(), "libsupervisor.so")
  subprocess.check_call(["gcc", "-shared", "-fPIC", "-std=gnu11", "-Wall", "-Wextra", "-Werror", "-I", PANDA,
                         os.path.join(HERE, "shim.c"), "-o", out])
  lib = ctypes.CDLL(out)
  sig = {
    "redundant_set": ([ctypes.POINTER(Redundant), ctypes.c_uint32], None),
    "redundant_valid": ([ctypes.POINTER(Redundant)], ctypes.c_bool),
    "supervisor_init": ([ctypes.POINTER(Supervisor)], None),
    "supervisor_report_faults": ([ctypes.POINTER(Supervisor), ctypes.c_uint32], None),
    "supervisor_output_allowed": ([ctypes.POINTER(Supervisor)], ctypes.c_bool),
    "supervisor_checkpoint": ([ctypes.POINTER(Supervisor), ctypes.c_uint32], None),
    "supervisor_housekeeping_done": ([ctypes.POINTER(Supervisor)], None),
    "supervisor_tick": ([ctypes.POINTER(Supervisor)], ctypes.c_bool),
    "code_crc_init": ([ctypes.POINTER(CodeCrc), ctypes.c_void_p, ctypes.c_uint32], None),
    "code_crc_step": ([ctypes.POINTER(CodeCrc), ctypes.c_uint32], ctypes.c_uint8),
    "march_c_minus": ([ctypes.POINTER(ctypes.c_uint32), ctypes.c_uint32], ctypes.c_bool),
    "shim_escalated_faults": ([], ctypes.c_uint32),
    "shim_crc32": ([ctypes.c_char_p, ctypes.c_uint32], ctypes.c_uint32),
  }
  for name, (args, res) in sig.items():
    fn = getattr(lib, name)
    fn.argtypes, fn.restype = args, res
  return lib


LIB = build()


class TestRedundant(unittest.TestCase):
  def test_set_and_corrupt(self):
    r = Redundant()
    self.assertFalse(LIB.redundant_valid(r), "a zeroed copy must be invalid")
    for v in (0, 1, 0xDEADBEEF, 0xFFFFFFFF):
      LIB.redundant_set(r, v)
      self.assertTrue(LIB.redundant_valid(r))
      for bit in range(32):
        r.value ^= 1 << bit
        self.assertFalse(LIB.redundant_valid(r))
        r.value ^= 1 << bit
        r.inverse ^= 1 << bit
        self.assertFalse(LIB.redundant_valid(r))
        r.inverse ^= 1 << bit


class TestFaultEscalation(unittest.TestCase):
  ESCALATED = ["FAULT_UNUSED_INTERRUPT_HANDLED", "FAULT_INTERRUPT_RATE_CAN_1", "FAULT_INTERRUPT_RATE_CAN_2",
               "FAULT_INTERRUPT_RATE_CAN_3", "FAULT_INTERRUPT_RATE_INTERRUPTS", "FAULT_INTERRUPT_RATE_TICK",
               "FAULT_INTERRUPT_RATE_SPI", "FAULT_INTERRUPT_RATE_SPI_DMA", "FAULT_REGISTER_DIVERGENT",
               "FAULT_HEARTBEAT_LOOP_WATCHDOG", "FAULT_ECC_UNCORRECTABLE", "FAULT_SAFETY_STATE_CORRUPT",
               "FAULT_CODE_CRC_MISMATCH", "FAULT_STARTUP_RAM_TEST"]

  def setUp(self):
    self.s = Supervisor()
    LIB.supervisor_init(self.s)

  def test_mask_is_exactly_the_reviewed_set(self):
    self.assertEqual(LIB.shim_escalated_faults(), sum(FAULTS[f] for f in self.ESCALATED))

  def test_record_only_faults_do_not_block(self):
    for name, bit in FAULTS.items():
      if name in self.ESCALATED:
        continue
      LIB.supervisor_report_faults(self.s, bit)
      self.assertTrue(LIB.supervisor_output_allowed(self.s), name)

  def test_each_escalated_fault_latches_until_power_cycle(self):
    for name in self.ESCALATED:
      s = Supervisor()
      LIB.supervisor_init(s)
      self.assertTrue(LIB.supervisor_output_allowed(s))
      LIB.supervisor_report_faults(s, FAULTS[name])
      self.assertFalse(LIB.supervisor_output_allowed(s), name)
      for _ in range(100):  # the fault bit clearing does not unlatch
        LIB.supervisor_report_faults(s, 0)
        LIB.supervisor_tick(s)
      self.assertFalse(LIB.supervisor_output_allowed(s), name)

  def test_uninitialised_state_blocks_output(self):
    self.assertFalse(LIB.supervisor_output_allowed(Supervisor()))

  def test_corrupted_latch_blocks_and_records_corruption(self):
    self.s.latched_faults.inverse ^= 1 << 7
    self.assertFalse(LIB.supervisor_output_allowed(self.s))
    LIB.supervisor_report_faults(self.s, 0)
    self.assertFalse(LIB.supervisor_output_allowed(self.s))
    self.assertTrue(LIB.redundant_valid(self.s.latched_faults))
    self.assertEqual(self.s.latched_faults.value, FAULTS["FAULT_SAFETY_STATE_CORRUPT"])

  def test_flipped_latch_value_cannot_unlatch(self):
    LIB.supervisor_report_faults(self.s, FAULTS["FAULT_REGISTER_DIVERGENT"])
    self.s.latched_faults.value = 0  # a bit flip back to "no fault"
    self.assertFalse(LIB.supervisor_output_allowed(self.s))


class TestProgramFlow(unittest.TestCase):
  def setUp(self):
    self.s = Supervisor()
    LIB.supervisor_init(self.s)

  def run_ticks(self, n, main_loop=True, housekeeping=True, start=0):
    results = []
    for i in range(start, start + n):
      if main_loop:
        LIB.supervisor_checkpoint(self.s, CP_MAIN_LOOP)
      if housekeeping and i % 8 == 0:
        LIB.supervisor_housekeeping_done(self.s)
      results.append(LIB.supervisor_tick(self.s))
    return results

  def test_healthy_cadence_always_refreshes(self):
    self.assertTrue(all(self.run_ticks(800)))

  def test_stalled_main_loop_stops_refresh(self):
    self.assertTrue(all(self.run_ticks(16)))
    self.assertFalse(any(self.run_ticks(4, main_loop=False, start=16)))

  def test_main_loop_checkpoint_is_consumed_each_tick(self):
    LIB.supervisor_checkpoint(self.s, CP_MAIN_LOOP)
    LIB.supervisor_housekeeping_done(self.s)
    self.assertTrue(LIB.supervisor_tick(self.s))
    self.assertFalse(LIB.supervisor_tick(self.s))

  def test_missed_housekeeping_stops_refresh_on_the_tick_it_was_due(self):
    self.assertTrue(all(self.run_ticks(8)))  # housekeeping at tick 0, then 7 more ticks
    results = self.run_ticks(4, housekeeping=False, start=8)  # tick 8 is due and skipped
    self.assertEqual(results, [False, False, False, False])

  def test_refresh_does_not_depend_on_the_fault_latch(self):
    LIB.supervisor_report_faults(self.s, FAULTS["FAULT_ECC_UNCORRECTABLE"])
    self.assertTrue(all(self.run_ticks(80)))


class TestCrc(unittest.TestCase):
  def test_matches_zlib(self):
    rng = random.Random(1)
    for n in (0, 1, 3, 4, 255, 4096, 10240):
      data = bytes(rng.randrange(256) for _ in range(n))
      self.assertEqual(LIB.shim_crc32(data, n), zlib.crc32(data))

  def test_code_integrity_detects_a_flipped_bit(self):
    rng = random.Random(2)
    image = (ctypes.c_uint8 * 82220)(*[rng.randrange(256) for _ in range(82220)])
    st = CodeCrc()
    LIB.code_crc_init(st, ctypes.addressof(image), len(image))

    def full_pass():
      for _ in range(100):
        r = LIB.code_crc_step(st, 10240)
        if r != CODE_CRC_BUSY:
          return r
      raise AssertionError("pass did not complete")

    self.assertEqual(full_pass(), CODE_CRC_OK)  # sets the reference
    self.assertEqual(st.reference.value, zlib.crc32(bytes(image)))
    self.assertEqual(full_pass(), CODE_CRC_OK)
    image[40000] ^= 0x10
    self.assertEqual(full_pass(), CODE_CRC_MISMATCH)
    image[40000] ^= 0x10
    self.assertEqual(full_pass(), CODE_CRC_OK)
    st.reference.inverse ^= 1  # corrupted reference is a mismatch, not a pass
    self.assertEqual(full_pass(), CODE_CRC_MISMATCH)

  def test_pass_takes_one_second_at_8hz(self):
    st = CodeCrc()
    image = (ctypes.c_uint8 * 82220)()
    LIB.code_crc_init(st, ctypes.addressof(image), len(image))
    steps = 1
    while LIB.code_crc_step(st, 10240) == CODE_CRC_BUSY:
      steps += 1
    self.assertLessEqual(steps, 9)


class TestMarch(unittest.TestCase):
  def test_good_memory_passes_and_is_left_zeroed(self):
    mem = (ctypes.c_uint32 * 4096)(*range(4096))
    self.assertTrue(LIB.march_c_minus(mem, len(mem)))
    self.assertEqual(list(mem), [0] * 4096)

  def test_zero_length(self):
    mem = (ctypes.c_uint32 * 1)(5)
    self.assertTrue(LIB.march_c_minus(mem, 0))
    self.assertEqual(mem[0], 5)


if __name__ == "__main__":
  unittest.main()
