"""LionDriver HIL test: Toyota RX timeout and engagement gate (TSR-401, TSR-306).

Verifies SWSR-401/401a and SWSR-306 on the device (LionDriver WP-V-05). Needs:
  - a panda flashed with the LionDriver firmware (any build; 100 Hz safety task, opendbc with the 5-period
    Toyota RX timeout)
  - a panda jungle on the default harness orientation, which sends the Toyota RX messages on bus 0
  - HITL=1 in the environment; REFERENCE_BUILD=1 for a LIONDRIVER_REFERENCE=1 build, whose Toyota RX set also
    contains EPS_STATUS and PCM_CRUISE_2 (TSR-110, TSR-401)

The jungle plays the car: it sends the four Toyota RX check messages at their nominal rates, the test engages
with a PCM cruise rising edge, then stops one message and measures the time until controls_allowed drops.
"""
import os
import threading
import time

from opendbc.can.packer import CANPacker
from opendbc.car.structs import CarParams
from panda.tests.hitl.base import PandaTestCase, panda_test

TOYOTA = CarParams.SafetyModel.toyota
SILENT = CarParams.SafetyModel.silent
REFERENCE_PARAM = 73
DBC = "toyota_nodsu_pt_generated"

# message name -> nominal rate (Hz), as declared in opendbc/safety/modes/toyota.h
RX_MSGS = {"WHEEL_SPEEDS": 83, "STEER_TORQUE_SENSOR": 50, "PCM_CRUISE": 33, "BRAKE_MODULE": 40}
if os.environ.get("REFERENCE_BUILD") == "1":
  RX_MSGS |= {"EPS_STATUS": 25, "PCM_CRUISE_2": 33}
TIMEOUT_PERIODS = 5
SAFETY_TASK_PERIOD_S = 0.010
# TSR-401: worst-case detection <= 0.2 s. The measurement adds USB polling of the health packet and the
# jungle's send jitter on top of the firmware's detection time.
TSR401_BUDGET_S = 0.2
POLL_TIMEOUT_S = 1.5


# LionDriver TSR-408: while authority is granted, the reference build revokes it when STEERING_LKA (50 ms) or
# ACC_CONTROL (100 ms) stops arriving from the SoC, so a test that keeps authority sends them like the SoC does
SOC_STEP_S = 0.010     # STEERING_LKA at 100 Hz, as the SoC sends it (and within its TSR-518 rate budget)
ACC_CONTROL_EVERY = 3  # steps: 33 Hz, within the ACC_CONTROL rate budget


class SocSim:
  """Plays the engaged SoC from the host side: heartbeat, STEERING_LKA and ACC_CONTROL. Each call to step() waits
  until 10 ms after the previous one, so a polling loop that calls it never sends faster than the SoC does."""

  def __init__(self, panda):
    self.panda = panda
    self.packer = CANPacker(DBC)
    self.steps = 0
    self.next_step = 0.0

  def step(self, heartbeat=True):
    time.sleep(max(0.0, self.next_step - time.monotonic()))
    self.next_step = time.monotonic() + SOC_STEP_S
    if heartbeat:
      self.panda.send_heartbeat(engaged=True)
    self.panda.can_send(*self.packer.make_can_msg("STEERING_LKA", 0, {"STEER_TORQUE_CMD": 0, "STEER_REQUEST": 0}))
    if self.steps % ACC_CONTROL_EVERY == 0:
      self.panda.can_send(*self.packer.make_can_msg("ACC_CONTROL", 0, {"ACCEL_CMD": 0.0}))
    self.steps += 1


class CarSim:
  """Sends the Toyota RX check messages from the jungle at their nominal rates on bus 0."""

  def __init__(self, jungle):
    self.jungle = jungle
    self.packer = CANPacker(DBC)
    self.cruise_active = False
    self.lka_state = 5  # active
    self.stopped = set()
    self._run = threading.Event()
    self._thread = None

  def _msg(self, name):
    values = {}
    if name == "PCM_CRUISE":
      values = {"CRUISE_ACTIVE": int(self.cruise_active), "GAS_RELEASED": 1}
    elif name == "EPS_STATUS":
      values = {"LKA_STATE": self.lka_state}
    addr, dat, bus = self.packer.make_can_msg(name, 0, values)
    return addr, dat, bus

  def _loop(self):
    next_due = {name: time.monotonic() for name in RX_MSGS}
    while self._run.is_set():
      now = time.monotonic()
      batch = []
      for name, hz in RX_MSGS.items():
        if name not in self.stopped and now >= next_due[name]:
          batch.append(self._msg(name))
          next_due[name] += 1.0 / hz
      if batch:
        self.jungle.can_send_many(batch)
      time.sleep(0.001)

  def start(self):
    self._run.set()
    self._thread = threading.Thread(target=self._loop, daemon=True)
    self._thread.start()

  def stop(self):
    self._run.clear()
    if self._thread is not None:
      self._thread.join()


@panda_test(needs_jungle=True)
class TestToyotaRxTimeout(PandaTestCase):
  def setUp(self):
    super().setUp()
    from panda.tests.hitl.base import _panda_jungle
    self.sim = CarSim(_panda_jungle)
    self.soc = SocSim(self.p)
    self.p.set_safety_mode(TOYOTA, REFERENCE_PARAM)
    self.sim.start()
    self.addCleanup(self.p.set_safety_mode, SILENT)
    self.addCleanup(self.sim.stop)
    time.sleep(0.5)

  def _engage(self):
    # keep the heartbeat younger than 0.3 s throughout (TSR-407), or the panda revokes authority
    self.p.send_heartbeat(engaged=False)
    self.sim.cruise_active = False
    time.sleep(0.1)
    self.p.send_heartbeat(engaged=False)
    self.sim.cruise_active = True
    time.sleep(0.1)
    self.p.send_heartbeat(engaged=True)
    return self.p.health()["controls_allowed"]

  def test_engages_with_all_messages(self):
    self.assertTrue(self._engage(), "no engagement with every RX message present")

  def test_timeout_per_message(self):
    for name, hz in RX_MSGS.items():
      with self.subTest(msg=name):
        self.sim.stopped.clear()
        self.assertTrue(self._engage(), "no engagement before the dropout")
        firmware_bound = TIMEOUT_PERIODS / hz + SAFETY_TASK_PERIOD_S

        t0 = time.monotonic()
        self.sim.stopped.add(name)
        elapsed = None
        while time.monotonic() - t0 < POLL_TIMEOUT_S:
          self.soc.step()  # the SoC keeps commanding, so only the RX check can revoke
          if not self.p.health()["controls_allowed"]:
            elapsed = time.monotonic() - t0
            break
        print(f"{name} ({hz} Hz): revoked after {elapsed} s (firmware bound {firmware_bound:.3f} s)")
        self.assertIsNotNone(elapsed, f"controls not revoked within {POLL_TIMEOUT_S} s of losing {name}")
        self.assertLess(elapsed, TSR401_BUDGET_S)
        self.sim.stopped.clear()
        time.sleep(0.2)

  def test_no_engagement_while_message_missing(self):
    for name in RX_MSGS:
      if name == "PCM_CRUISE":
        continue  # the rising edge itself is carried by PCM_CRUISE
      with self.subTest(msg=name):
        self.sim.stopped = {name}
        time.sleep(0.2)
        self.assertFalse(self._engage(), f"engaged while {name} was missing")
        self.sim.stopped.clear()
        time.sleep(0.2)


if __name__ == "__main__":
  import unittest
  unittest.main()
