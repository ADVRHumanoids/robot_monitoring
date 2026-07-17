"""Tests for serving the packaged Vue monitor entrypoint."""

import asyncio
import sys
from pathlib import Path

from aiohttp.test_utils import TestClient, TestServer


SERVER_SRC = Path(__file__).parents[1] / "src"
PROTO_SRC = SERVER_SRC / "xbot2_gui_server" / "proto"
sys.path[:0] = [str(SERVER_SRC), str(PROTO_SRC)]

from xbot2_gui_server.server import Xbot2WebServer  # noqa: E402


def test_monitor_routes_serve_index_instead_of_directory_listing():
    async def scenario():
        server = Xbot2WebServer()
        client = TestClient(TestServer(server.app))
        await client.start_server()
        try:
            root_response = await client.get("/", allow_redirects=False)
            monitor_response = await client.get("/monitor", allow_redirects=False)
            index_response = await client.get("/monitor/")
            index_html = await index_response.text()
        finally:
            await client.close()

        assert root_response.status == 302
        assert root_response.headers["Location"] == "/monitor/"
        assert monitor_response.status == 302
        assert monitor_response.headers["Location"] == "/monitor/"
        assert index_response.status == 200
        assert "<div id=\"app\"></div>" in index_html
        assert "Index of /monitor" not in index_html

    asyncio.run(scenario())
