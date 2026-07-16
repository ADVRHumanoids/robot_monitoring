"""Transport compatibility test without starting ROS or an HTTP listener."""

import asyncio
import sys
from pathlib import Path


SERVER_SRC = Path(__file__).parents[1] / "src"
PROTO_SRC = SERVER_SRC / "xbot2_gui_server" / "proto"
sys.path[:0] = [str(SERVER_SRC), str(PROTO_SRC)]

from xbot2_gui_server.server import Xbot2WebServer  # noqa: E402


class FakeWebSocket:
    def __init__(self):
        self.messages = []

    async def send_bytes(self, message):
        self.messages.append(message)


class FakeUdpSocket:
    def __init__(self):
        self.messages = []

    def sendto(self, message, address):
        self.messages.append((message, address))


def test_udp_broadcast_is_mirrored_only_to_opted_in_websockets():
    server = Xbot2WebServer.__new__(Xbot2WebServer)
    tunnel_client = FakeWebSocket()
    regular_client = FakeWebSocket()
    udp_client = ("127.0.0.1", 9000)

    server.ws_clients = {tunnel_client, regular_client}
    server.ws_udp_tunnel = {tunnel_client}
    server.udp = FakeUdpSocket()
    server.udp_clients = {udp_client}
    server.udp_msg_seq = 0

    asyncio.run(server.udp_send_to_all({"type": "proc_status", "name": "core"}))

    assert len(tunnel_client.messages) == 1
    assert regular_client.messages == []
    assert len(server.udp.messages) == 1
    assert server.udp.messages[0][1] == udp_client
