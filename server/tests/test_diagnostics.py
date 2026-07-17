"""Tests for forwarding aggregated ROS diagnostics to frontend clients."""

import asyncio
import sys
from pathlib import Path

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue


SERVER_SRC = Path(__file__).parents[1] / 'src'
sys.path.insert(0, str(SERVER_SRC))

import xbot2_gui_server.diagnostics as diagnostics  # noqa: E402


def make_diagnostics_message():
    """Build a representative ROS diagnostics message."""
    message = DiagnosticArray()
    message.header.stamp.sec = 12
    message.header.stamp.nanosec = 500_000_000
    message.header.frame_id = 'base_link'
    message.status = [
        DiagnosticStatus(
            level=DiagnosticStatus.WARN,
            name='motors/left_knee',
            message='temperature high',
            hardware_id='left_knee_driver',
            values=[KeyValue(key='temperature', value='82.5')],
        )
    ]
    return message


def test_diagnostic_array_to_dict_preserves_ros_message_fields():
    """All useful DiagnosticArray fields are retained in the JSON payload."""
    assert diagnostics.diagnostic_array_to_dict(make_diagnostics_message()) == {
        'type': 'diagnostics',
        'stamp': 12.5,
        'frame_id': 'base_link',
        'status': [
            {
                'level': 1,
                'name': 'motors/left_knee',
                'message': 'temperature high',
                'hardware_id': 'left_knee_driver',
                'values': [{'key': 'temperature', 'value': '82.5'}],
            }
        ],
    }


def test_handler_forwards_received_messages():
    """A received ROS sample is sent through the frontend UDP transport."""
    class FakeRosHandle:

        def create_subscription(self, msg_class, topic, callback, queue_size):
            self.subscription = (msg_class, topic, callback, queue_size)
            return self.subscription

    class FakeServer:

        def __init__(self):
            self.messages = []

        def schedule_task(self, coroutine):
            return asyncio.create_task(coroutine)

        async def udp_send_to_all(self, message):
            self.messages.append(message)

    async def scenario():
        fake_ros = FakeRosHandle()
        fake_server = FakeServer()
        original_ros_handle = diagnostics.ros_handle
        diagnostics.ros_handle = fake_ros
        try:
            handler = diagnostics.DiagnosticsHandler(
                fake_server,
                {'rate': 1000.0, 'queue_size': 2},
            )
            handler.on_diagnostics_recv(make_diagnostics_message())
            await asyncio.sleep(0.01)
            handler.task.cancel()
            await asyncio.gather(handler.task, return_exceptions=True)
        finally:
            diagnostics.ros_handle = original_ros_handle

        assert fake_ros.subscription[1:] == (
            '/diagnostics_agg',
            handler.on_diagnostics_recv,
            2,
        )
        assert fake_server.messages == [
            diagnostics.diagnostic_array_to_dict(make_diagnostics_message())
        ]

    asyncio.run(scenario())
