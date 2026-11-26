#!/usr/bin/env python3
"""Host tests for the SPI protocol version 3 (LionDriver TSR-410, 411, 412; SWSR-410, 410a, 412; WP-S-05 §4.1).

board/drivers/spi.h is compiled for the host with a simulated SPI4 bus: every byte the host clocks goes into the armed
RX DMA or comes out of the armed TX DMA, and the DMA-complete and end-of-transfer interrupts run between bytes. The
host side is the real python/spi.py PandaSpiHandle, loaded without the rest of the library, so both ends of the link
are the shipped code. Timing on the real bus (50 MHz, interrupt latency) and the DMA hardware are checked on the HIL
bench (tests/hitl/test_5_spi.py), not here.
"""
import ctypes
import importlib.util
import os
import random
import struct
import subprocess
import sys
import tempfile
import types
import unittest
from contextlib import contextmanager

PANDA_DIR = os.path.realpath(os.path.join(os.path.dirname(__file__), "..", ".."))
OPENDBC_DIR = os.path.realpath(os.path.join(PANDA_DIR, "..", "opendbc_repo"))  # the LionDriver superproject's pin
if not os.path.isdir(OPENDBC_DIR):
  import opendbc  # panda on its own: the installed opendbc
  OPENDBC_DIR = opendbc.INCLUDE_PATH

HARNESS = os.path.join(os.path.dirname(os.path.realpath(__file__)), "harness.c")


def load_spi_module():
  """Import python/spi.py and its stdlib-only siblings without python/__init__.py (which needs USB and opendbc)."""
  name = "liondriver_pandapy"
  if name + ".spi" in sys.modules:
    return sys.modules[name + ".spi"]
  pkg = types.ModuleType(name)
  pkg.__path__ = [os.path.join(PANDA_DIR, "python")]
  sys.modules[name] = pkg
  for sub in ("constants", "utils", "base", "spi"):
    spec = importlib.util.spec_from_file_location(f"{name}.{sub}", os.path.join(PANDA_DIR, "python", f"{sub}.py"))
    mod = importlib.util.module_from_spec(spec)
    sys.modules[f"{name}.{sub}"] = mod
    spec.loader.exec_module(mod)
  return sys.modules[name + ".spi"]


class Bus:
  """A spidev stand-in clocking bytes through the compiled firmware; `corrupt` may flip bits on either line."""

  def __init__(self, lib):
    self.lib = lib
    self.mosi_hook = None  # f(list of bytes) -> list of bytes, applied to one host write
    self.miso_hook = None

  def _clock(self, data):
    data = list(data)
    if self.mosi_hook is not None:
      data = self.mosi_hook(data)
    out = [self.lib.bus_byte(b & 0xFF) for b in data]
    self.lib.end_transaction()
    if self.miso_hook is not None:
      out = self.miso_hook(out)
    return out

  def xfer2(self, data):
    return self._clock(data)

  def writebytes(self, data):
    self._clock(data)

  def readbytes(self, length):
    return bytes(self._clock([0] * length))


class FakeDev:
  def __init__(self, bus):
    self.bus = bus

  @contextmanager
  def acquire(self):
    yield self.bus


class TestSpiProtocol(unittest.TestCase):
  @classmethod
  def setUpClass(cls):
    cls.tmp = tempfile.TemporaryDirectory()
    lib = os.path.join(cls.tmp.name, "libspi.so")
    subprocess.check_call(["gcc", "-std=gnu11", "-shared", "-fPIC", "-Wall", "-Wextra", "-Werror", "-Wno-unused-function",
                           "-I", PANDA_DIR, "-I", OPENDBC_DIR, "-o", lib, HARNESS])
    cls.lib = ctypes.CDLL(lib)
    cls.lib.bus_byte.restype = ctypes.c_uint8
    cls.lib.bus_byte.argtypes = [ctypes.c_uint8]
    cls.lib.errors.restype = ctypes.c_uint16
    cls.lib.crc32c_of.restype = ctypes.c_uint32
    cls.lib.crc32c_of.argtypes = [ctypes.c_char_p, ctypes.c_uint32]
    cls.spi = load_spi_module()

  @classmethod
  def tearDownClass(cls):
    cls.tmp.cleanup()

  def setUp(self):
    self.lib.reset_mcu()
    self.bus = Bus(self.lib)
    self.host = self.new_host()

  def new_host(self):
    h = self.spi.PandaSpiHandle.__new__(self.spi.PandaSpiHandle)
    h.dev = FakeDev(self.bus)
    h.no_retry = False
    import threading
    h._seq_lock = threading.Lock()
    h._seq = 0
    return h

  def counter(self, name):
    return ctypes.c_int.in_dll(self.lib, name).value

  # raw host for fault cases: one transfer with explicit fields, returning the panda's reply bytes
  def raw_transfer(self, endpoint, payload, seq, frame=0, max_rx=64, corrupt_data=None):
    spi = self.spi
    header = struct.pack("<BBHH", spi.SYNC, endpoint, len(payload), max_rx)
    self.bus.xfer2(header + bytes([spi.crc8(header, spi.HEADER_CRC_POLY)]))
    hack = self.bus.xfer2([0x11])
    self.assertEqual(hack[0], spi.HACK)
    body = bytes([seq, frame, *payload])
    data = bytearray(body + struct.pack("<I", spi.crc32c(header + body)))
    if corrupt_data is not None:
      data = corrupt_data(data)
    self.bus.xfer2(data)
    # read exactly the reply, so no stray bytes reach the panda's next header
    reply = self.bus.xfer2([0x13])
    if reply[0] == spi.DACK:
      reply += self.bus.xfer2([0x13] * (spi.RESP_HEADER_SIZE - 1))
      reply += self.bus.xfer2([0x13] * (struct.unpack("<H", bytes(reply[1:3]))[0] + spi.CRC_SIZE))
    return bytes(reply)

  # --- CRC-32C (SWSR-410) ---

  def test_crc32c_check_value_firmware_and_host(self):
    self.assertEqual(self.lib.crc32c_of(b"123456789", 9), 0xE3069283)
    self.assertEqual(self.spi.crc32c(b"123456789"), 0xE3069283)
    rng = random.Random(1)
    for n in (0, 1, 2, 7, 64, 325, 2041):
      d = bytes(rng.randrange(256) for _ in range(n))
      self.assertEqual(self.lib.crc32c_of(d, n), self.spi.crc32c(d), n)

  def test_protocol_version_3_both_ends(self):
    self.assertEqual(self.lib.protocol_version(), 3)
    self.assertEqual(self.spi.PandaSpiHandle.PROTOCOL_VERSION, 3)
    self.assertEqual(self.lib.max_mosi_len(), self.spi.MAX_MOSI_LEN)
    self.assertEqual(self.lib.max_miso_len(), self.spi.MAX_MISO_LEN)

  def test_version_request_unchanged(self):
    # the 7-byte VERSION request is how a host finds the protocol version of any firmware, so it must not change
    dat = self.host.get_protocol_version()
    self.assertEqual(dat[:12], bytes(range(1, 13)))
    self.assertEqual(dat[14], 3)
    self.assertEqual(self.lib.errors(), 0)

  # --- end to end through the library ---

  def test_control_read_and_write(self):
    self.assertEqual(self.host.controlRead(0, 0xd2, 0, 0, 10), bytes((0xd2 + i) & 0xFF for i in range(10)))
    self.host.controlWrite(0, 0xdc, 0, 0, b"")
    self.assertEqual(self.counter("control_calls"), 2)
    self.assertEqual(self.lib.errors(), 0)

  def test_can_write_and_read(self):
    self.host.bulkWrite(3, bytes(range(100)))
    self.assertEqual(self.counter("can_write_calls"), 1)
    ctypes.c_int.in_dll(self.lib, "can_read_avail").value = 300
    self.assertEqual(self.host.bulkRead(1, 1000), bytes((0xA0 + i) & 0xFF for i in range(300)))
    self.assertEqual(self.lib.errors(), 0)

  def test_long_transfers_split_within_bound(self):
    self.host.bulkWrite(3, bytes(5000))
    self.assertLessEqual(self.counter("max_mosi_dma"), 2047)  # TSR-412: data phase at most 2047 bytes
    self.assertEqual(self.lib.errors(), 0)

  def test_counter_wraps_without_zero(self):
    for _ in range(600):
      self.host.controlWrite(0, 0xdc, 0, 0, b"")
    self.assertEqual(self.counter("control_calls"), 600)
    self.assertEqual(self.lib.errors(), 0)

  # --- corruption (SWSR-410: all 1-3 bit errors and bursts up to 32 bits detected) ---

  def corrupted_writes_executed(self, flips):
    """Send one CAN write whose data phase is corrupted by `flips`; True if the panda executed it."""
    before = self.counter("can_write_calls")
    payload = bytes(random.Random(len(flips)).randrange(256) for _ in range(64))
    seq = 1 + (self.counter("can_write_calls") % 200)
    def corrupt(data):
      for bit in flips:
        data[bit // 8] ^= 1 << (bit % 8)
      return data
    self.lib.reset_mcu()
    reply = self.raw_transfer(3, payload, seq, corrupt_data=corrupt)
    executed = self.counter("can_write_calls") != before and self.counter("can_write_calls") > 0
    if not executed:
      self.assertEqual(reply[0], self.spi.NACK)
      self.assertEqual(self.lib.errors(), 1)
    return executed

  def test_single_and_double_bit_errors_detected(self):
    nbits = (2 + 64 + 4) * 8
    for a in range(nbits):
      self.assertFalse(self.corrupted_writes_executed([a]), a)
    rng = random.Random(2)
    for _ in range(3000):
      flips = rng.sample(range(nbits), 2)
      self.assertFalse(self.corrupted_writes_executed(flips), flips)

  def test_triple_bit_errors_detected(self):
    nbits = (2 + 64 + 4) * 8
    rng = random.Random(3)
    for _ in range(3000):
      flips = rng.sample(range(nbits), 3)
      self.assertFalse(self.corrupted_writes_executed(flips), flips)

  def test_burst_errors_up_to_32_bits_detected(self):
    nbits = (2 + 64 + 4) * 8
    rng = random.Random(4)
    for _ in range(3000):
      length = rng.randint(2, 32)
      start = rng.randrange(nbits - length + 1)
      # a burst starts and ends with a flipped bit; the bits between are random
      inner = [start + i for i in range(1, length - 1) if rng.random() < 0.5]
      flips = [start, start + length - 1] + inner
      self.assertFalse(self.corrupted_writes_executed(flips), flips)

  def test_corrupted_header_refused_before_data(self):
    spi = self.spi
    header = bytearray(struct.pack("<BBHH", spi.SYNC, 3, 64, 64))
    crc = spi.crc8(bytes(header), spi.HEADER_CRC_POLY)
    header[2] ^= 0x10  # MOSI length 64 -> 80
    self.bus.xfer2(bytes(header) + bytes([crc]))
    self.assertEqual(self.bus.xfer2([0x11])[0], spi.NACK)
    self.assertEqual(self.lib.errors(), 1)

  def test_corrupted_response_detected_by_host(self):
    flipped = []
    def miso_hook(out):
      if not flipped and out and out[0] == self.spi.DACK:
        out[5] ^= 0x04
        flipped.append(True)
      return out
    self.bus.miso_hook = miso_hook
    with self.assertRaises(self.spi.PandaSpiBadChecksum):
      self.host._transfer_spidev(self.bus, 0, struct.pack("<BHHH", 0xd2, 0, 0, 8), 100, max_rx_len=8, seq=0)

  # --- lengths (SWSR-412) ---

  def length_header(self, mosi, miso):
    spi = self.spi
    header = struct.pack("<BBHH", spi.SYNC, 0xAB, mosi, miso)
    self.bus.xfer2(header + bytes([spi.crc8(header, spi.HEADER_CRC_POLY)]))
    return self.bus.xfer2([0x11])[0]

  def test_length_bounds(self):
    spi = self.spi
    self.assertEqual(self.length_header(spi.MAX_MOSI_LEN, 0), spi.HACK)
    self.lib.reset_mcu()
    self.assertEqual(self.length_header(spi.MAX_MOSI_LEN + 1, 0), spi.NACK)
    self.lib.reset_mcu()
    self.assertEqual(self.length_header(0, spi.MAX_MISO_LEN), spi.HACK)
    self.lib.reset_mcu()
    self.assertEqual(self.length_header(0, spi.MAX_MISO_LEN + 1), spi.NACK)
    self.lib.reset_mcu()
    self.assertEqual(self.length_header(0xFFFF, 0xFFFF), spi.NACK)
    self.assertEqual(self.lib.errors(), 1)
    self.assertEqual(self.counter("max_mosi_dma"), 7)  # only header DMAs were started

  def test_test_endpoint_read_bounded(self):
    # 0xAB answers with as many bytes as the host asks for: the bound keeps it inside spi_buf_tx
    reply = self.raw_transfer(0xAB, b"", 1, max_rx=self.spi.MAX_MISO_LEN)
    self.assertEqual(reply[0], self.spi.DACK)
    self.assertEqual(struct.unpack("<H", reply[1:3])[0], self.spi.MAX_MISO_LEN)

  # --- sequence counter (SWSR-410a) ---

  def test_repeated_transfer_not_executed_again(self):
    payload = bytes(16)
    self.assertEqual(self.raw_transfer(3, payload, 0)[0], self.spi.DACK)
    self.lib.tx_ready()
    reply = self.raw_transfer(3, payload, 0)  # session start may repeat: 0 is always accepted
    self.assertEqual(reply[0], self.spi.DACK)
    self.lib.tx_ready()
    self.assertEqual(self.raw_transfer(3, payload, 1)[0], self.spi.DACK)
    self.lib.tx_ready()
    reply = self.raw_transfer(3, payload, 1)  # replay of the last executed transfer
    self.assertEqual(reply[0], self.spi.DACK)
    self.assertEqual(struct.unpack("<H", reply[1:3])[0], 0)
    self.assertEqual(reply[3], 1)
    self.assertEqual(self.counter("can_write_calls"), 3)
    self.assertEqual(self.lib.errors(), 1)

  def test_one_lost_transfer_tolerated(self):
    self.raw_transfer(0, struct.pack("<BHHH", 0xdc, 0, 0, 0), 5)
    self.assertEqual(self.raw_transfer(0, struct.pack("<BHHH", 0xdc, 0, 0, 0), 7)[0], self.spi.DACK)
    self.assertEqual(self.counter("control_calls"), 2)
    self.assertEqual(self.lib.errors(), 0)

  def test_gap_and_reorder_refused(self):
    ctrl = struct.pack("<BHHH", 0xdc, 0, 0, 0)
    self.raw_transfer(0, ctrl, 10)
    self.assertEqual(self.raw_transfer(0, ctrl, 13)[0], self.spi.SEQ_NACK)  # two lost: an insertion or a gap
    self.assertEqual(self.raw_transfer(0, ctrl, 9)[0], self.spi.SEQ_NACK)   # older than the last: reordered
    self.assertEqual(self.raw_transfer(0, ctrl, 11)[0], self.spi.DACK)
    self.assertEqual(self.counter("control_calls"), 2)
    self.assertEqual(self.lib.errors(), 2)

  def test_wrap_skips_zero(self):
    ctrl = struct.pack("<BHHH", 0xdc, 0, 0, 0)
    self.raw_transfer(0, ctrl, 254)
    self.assertEqual(self.raw_transfer(0, ctrl, 255)[0], self.spi.DACK)
    self.assertEqual(self.raw_transfer(0, ctrl, 1)[0], self.spi.DACK)
    self.assertEqual(self.raw_transfer(0, ctrl, 3)[0], self.spi.DACK)
    self.assertEqual(self.raw_transfer(0, ctrl, 2)[0], self.spi.SEQ_NACK)
    self.assertEqual(self.lib.errors(), 1)

  def test_nacked_transfer_does_not_advance_counter(self):
    # CAN TX buffer full: NACK without executing, the host retries with the same counter value
    self.raw_transfer(3, bytes(8), 20)
    self.lib.tx_full()
    self.assertEqual(self.raw_transfer(3, bytes(8), 21)[0], self.spi.NACK)
    self.lib.tx_ready()
    self.assertEqual(self.raw_transfer(3, bytes(8), 21)[0], self.spi.DACK)
    self.assertEqual(self.counter("can_write_calls"), 2)
    self.assertEqual(self.lib.errors(), 0)

  def test_second_host_session_resynchronises(self):
    for _ in range(5):
      self.host.controlWrite(0, 0xdc, 0, 0, b"")
    other = self.new_host()
    for _ in range(3):
      other.controlWrite(0, 0xdc, 0, 0, b"")  # starts at 0, panda follows
    # the first host is now out of step: one SEQ_NACK, then it restarts at 0 and goes on
    self.host.controlWrite(0, 0xdc, 0, 0, b"")
    self.host.controlWrite(0, 0xdc, 0, 0, b"")
    self.assertEqual(self.counter("control_calls"), 10)
    self.assertEqual(self.lib.errors(), 1)

  def test_lost_response_retry_not_executed_twice(self):
    # the panda executes a CAN write, its DACK is corrupted on the way back; the library retries with the same
    # counter value and the panda acknowledges without writing the frames a second time
    self.host.controlWrite(0, 0xdc, 0, 0, b"")
    hit = []
    def miso_hook(out):
      if not hit and out and out[0] == self.spi.DACK:
        out[4] ^= 0x01  # first CRC byte
        hit.append(True)
      return out
    self.bus.miso_hook = miso_hook
    self.host._transfer(3, bytes(range(32)), 1000)
    self.assertEqual(self.counter("can_write_calls"), 1)


if __name__ == "__main__":
  unittest.main()
