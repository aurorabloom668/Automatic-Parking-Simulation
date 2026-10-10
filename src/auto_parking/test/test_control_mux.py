import time

import pytest
import rclpy
from prius_msgs.msg import Control
from std_msgs.msg import String
from auto_parking.msg import ParkingStatus
from auto_parking.control_mux import ControlMux


@pytest.fixture
def mux():
    rclpy.init()
    node = ControlMux()
    yield node
    node.destroy_node()
    rclpy.shutdown()


def test_only_active_source_is_forwarded(mux):
    status = ParkingStatus(mode='MANUAL', state='MANUAL_APPROACH')
    manual = Control(throttle=0.35, shift_gears=Control.FORWARD)
    auto = Control(throttle=0.2, shift_gears=Control.REVERSE)
    mux._status(status)
    mux._auto(auto)
    assert mux.command is None
    mux._manual(manual)
    assert mux.command is manual
    mux._status(ParkingStatus(mode='AUTO', state='AUTO_PARKING'))
    assert mux.command is None
    mux._manual(manual)
    assert mux.command is None
    mux._auto(auto)
    assert mux.command is auto


@pytest.mark.parametrize('state', ['COMPLETE', 'HOLD_FOR_MANUAL', 'READY_TO_PARK'])
def test_non_driving_auto_states_brake(mux, state):
    mux._status(ParkingStatus(mode='AUTO', state=state))
    assert mux.source is None
    mux._auto(Control(throttle=0.2))
    assert mux.command is None


def test_cancel_immediately_clears_control(mux):
    mux._status(ParkingStatus(mode='AUTO', state='AUTO_PARKING'))
    mux._auto(Control(throttle=0.2))
    mux._request(String(data='cancel'))
    assert mux.source is None and mux.command is None


def test_start_clears_manual_command_before_auto_status(mux):
    mux._status(ParkingStatus(mode='MANUAL', state='READY_TO_PARK'))
    mux._manual(Control(throttle=0.35, shift_gears=Control.FORWARD))
    mux._request(String(data='start'))
    assert mux.source is None and mux.command is None
    mux._manual(Control(throttle=0.35, shift_gears=Control.FORWARD))
    assert mux.command is None
    mux._status(ParkingStatus(mode='AUTO', state='AUTO_PARKING'))
    assert mux.source == 'auto'


def test_stale_command_outputs_brake(mux):
    output = []
    publisher = mux.pub
    class Recorder:
        def publish(self, msg):
            output.append(msg)
    mux.pub = Recorder()
    mux._status(ParkingStatus(mode='MANUAL', state='MANUAL'))
    mux._manual(Control(throttle=0.35, shift_gears=Control.FORWARD))
    mux._tick()
    assert output[-1].throttle == 0.35
    mux.command_time = time.monotonic() - 1
    mux._tick()
    assert output[-1].brake == 1 and output[-1].shift_gears == Control.NEUTRAL
    mux.pub = publisher
