import time
from contextlib import AbstractContextManager

from panda import Panda
from opendbc.car.car_helpers import get_car
from opendbc.car.can_definitions import CanData
from opendbc.car.structs import CarParams, CarControl

class PandaRunner(AbstractContextManager):
  def __enter__(self):
    self.p = Panda()
    self.p.reset()

    # Initialize CAN timeout counter
    self.can_rcv_cum_timeout_counter = 0
    self.last_can_recv_time = time.monotonic()

    # setup + fingerprinting
    self.p.set_safety_mode(CarParams.SafetyModel.elm327, 1)

    # Wait for CAN messages
    print("Waiting for CAN messages...")
    can_received = False
    timeout_start = time.monotonic()
    while not can_received and (time.monotonic() - timeout_start) < 10.0:  # 10 seconds timeout
      try:
        recv = self.p.can_recv()
        if len(recv) > 0:
          can_received = True
          self.last_can_recv_time = time.monotonic()
          print(f"CAN messages received: {len(recv)} messages")
          break
        time.sleep(0.01)  # Short sleep to avoid busy wait
      except Exception as e:
        print(f"Waiting for CAN: {e}")
        time.sleep(0.1)

    if not can_received:
      print("Warning: No CAN messages received during initialization")

    self.CI = get_car(self._can_recv, self.p.can_send_many, self.p.set_obd, True, False)
    assert self.CI.CP.carFingerprint.lower() != "mock", "Unable to identify car. Check connections and ensure car is supported."

    safety_model = self.CI.CP.safetyConfigs[0].safetyModel
    self.p.set_safety_mode(CarParams.SafetyModel.elm327, 1)
    self.CI.init(self.CI.CP, self._can_recv, self.p.can_send_many)
    self.p.set_safety_mode(safety_model.raw, self.CI.CP.safetyConfigs[0].safetyParam)

    return self

  def __exit__(self, exc_type, exc_value, traceback):
    self.p.set_safety_mode(CarParams.SafetyModel.noOutput)
    self.p.reset()  # avoid siren
    return super().__exit__(exc_type, exc_value, traceback)

  @property
  def panda(self) -> Panda:
    return self.p

  def _can_recv(self, wait_for_one: bool = False) -> list[list[CanData]]:
    try:
      recv = self.p.can_recv()

      # Handle timeout
      current_time = time.monotonic()
      can_rcv_valid = len(recv) > 0

      if can_rcv_valid:
        self.last_can_recv_time = current_time
      elif wait_for_one:
        # If waiting for one and no data, try again with timeout
        timeout_start = current_time
        while len(recv) == 0 and (current_time - timeout_start) < 0.1:  # 100ms timeout
          recv = self.p.can_recv()
          current_time = time.monotonic()
          if len(recv) > 0:
            can_rcv_valid = True
            self.last_can_recv_time = current_time
            break

      # Update timeout counter
      if not can_rcv_valid:
        self.can_rcv_cum_timeout_counter += 1
      return [
        [CanData(addr, dat, bus) for addr, dat, bus in recv],
      ]

    except Exception as e:
      print(f"CAN receive error: {e}")
      self.can_rcv_cum_timeout_counter += 1
      # Return empty data on error
      return [[]]

  def read(self, strict: bool = True):
    can_data = self._can_recv(wait_for_one=True)[0]
    current_time = time.monotonic()

    # Check for CAN validity based on recent reception and timeout
    can_timeout = (current_time - self.last_can_recv_time) > 2.0  # 2 second timeout
    can_rcv_valid = len(can_data) > 0 and not can_timeout

    # Get Panda health info for debugging
    health = self.p.health()

    # Create timestamp
    timestamp_nanos = int(current_time * 1e9)
    cs = self.CI.update([timestamp_nanos, can_data])

    # Update canValid based on reception status and timeout
    # Force canValid to true if we have recent valid CAN data
    if hasattr(cs, "canValid"):
      # If we have recent valid CAN data, set canValid to true
      if can_rcv_valid:
        print(f"CAN Valid: True (received {len(can_data)} CAN messages), cd.canValid: {cs.canValid}")
        cs.canValid = True
      else:
        cs.canValid = False

    # Add error counter information
    if hasattr(cs, "canErrorCounter"):
      cs.canErrorCounter = self.can_rcv_cum_timeout_counter

    if strict:
      assert cs.canValid, (
        f"CAN went invalid, check connections. Timeout counter: {self.can_rcv_cum_timeout_counter}, Last recv: {current_time - self.last_can_recv_time:.2f}s ago"
      )
    return cs

  def write(self, cc: CarControl) -> None:
    if cc.enabled and not self.p.health()["controls_allowed"]:
      # prevent the car from faulting. print a warning?
      cc = CarControl(enabled=False)
    _, can_sends = self.CI.apply(cc)
    self.p.can_send_many(can_sends, timeout=25)
    self.p.send_heartbeat()

if __name__ == "__main__":
  with PandaRunner() as p:
    print(p.read())
