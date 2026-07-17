"""Forward aggregated ROS diagnostics to XBot2 GUI clients."""

import asyncio
from collections import deque

from diagnostic_msgs.msg import DiagnosticArray

from . import ros_utils
from .server import ServerBase


# ROS is initialized before extensions are imported by main.py.
ros_handle: ros_utils.RosWrapper = ros_utils.ros_handle


def _stamp_to_seconds(stamp) -> float:
    """Convert both ROS 1 and ROS 2 time messages to seconds."""
    if hasattr(stamp, 'sec'):
        return float(stamp.sec) + float(stamp.nanosec) * 1e-9
    if hasattr(stamp, 'to_sec'):
        return stamp.to_sec()
    return float(stamp.secs) + float(stamp.nsecs) * 1e-9


def _level_to_int(level) -> int:
    # ROS 2 represents uint8 fields as a single byte in some generated
    # message packages, while ROS 1 exposes the same field as an int.
    if isinstance(level, (bytes, bytearray)):
        return level[0]
    return int(level)


def diagnostic_array_to_dict(msg: DiagnosticArray) -> dict:
    """Convert a DiagnosticArray into the JSON frontend wire format."""
    return {
        'type': 'diagnostics',
        'stamp': _stamp_to_seconds(msg.header.stamp),
        'frame_id': msg.header.frame_id,
        'status': [
            {
                'level': _level_to_int(status.level),
                'name': status.name,
                'message': status.message,
                'hardware_id': status.hardware_id,
                'values': [
                    {'key': value.key, 'value': value.value}
                    for value in status.values
                ],
            }
            for status in msg.status
        ],
    }


class DiagnosticsHandler:
    """Subscribe to aggregated diagnostics and broadcast every received array."""

    def __init__(self, srv: ServerBase, config=None) -> None:
        """Create the diagnostics subscription and forwarding task."""
        config = config or {}

        self.srv = srv
        self.rate = float(config.get('rate', 10.0))
        self.topic = config.get('topic', '/diagnostics_agg')
        self.pending = deque(maxlen=int(config.get('queue_size', 10)))

        self.subscription = ros_handle.create_subscription(
            DiagnosticArray,
            self.topic,
            self.on_diagnostics_recv,
            queue_size=self.pending.maxlen,
        )
        self.task = self.srv.schedule_task(self.run())

    def on_diagnostics_recv(self, msg: DiagnosticArray) -> None:
        """Queue a ROS diagnostics sample for the asynchronous transport."""
        self.pending.append(msg)

    async def run(self) -> None:
        """Drain queued diagnostics into the frontend transport."""
        while True:
            await asyncio.sleep(1.0 / self.rate)

            while self.pending:
                msg = self.pending.popleft()
                await self.srv.udp_send_to_all(diagnostic_array_to_dict(msg))
