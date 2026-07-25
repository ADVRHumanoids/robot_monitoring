import asyncio
import functools
import json
from concurrent.futures import ThreadPoolExecutor
from dataclasses import dataclass, field
from typing import Any

from aiohttp import web

from . import ros_utils
from . import utils
from .proto import generic_pb2
from .server import ServerBase

ros_handle: ros_utils.RosWrapper = ros_utils.ros_handle

from mpeg_ts_image_transport_msgs.msg import MpegTsDatagram


@dataclass(slots=True)
class _WebSocketWriter:
    queue: asyncio.Queue[bytes]
    send_lock: asyncio.Lock
    task: asyncio.Task[None] | None = None


@dataclass(slots=True)
class _StreamState:
    input_queue: asyncio.Queue[tuple[int, bytes]]
    subscription: Any | None = None
    task: asyncio.Task[None] | None = None
    udp_clients: set[Any] = field(default_factory=set)
    ws_clients: dict[Any, _WebSocketWriter] = field(default_factory=dict)
    last_ros_seq: int = -1


class MpegTsVideoHandler:

    def __init__(self, srv: ServerBase, config: dict | None = None) -> None:
        config = config or {}

        self.srv = srv
        self.srv.register_ws_coroutine(self.handle_ws_msg)
        self.srv.add_route('GET', '/video/get_names', self.get_names_handler, 'video_get_names')

        # Keep both producer and client queues bounded. For a live video stream,
        # dropping stale packets is preferable to accumulating seconds of latency.
        self.input_queue_size = int(config.get('input_queue_size', 4))
        self.ws_queue_size = int(config.get('ws_queue_size', 256))
        self.send_timeout_sec = float(config.get('send_timeout_sec', 0.25))
        self.event_loop_yield_packets = int(config.get('event_loop_yield_packets', 32))
        worker_threads = int(config.get('worker_threads', 1))

        if self.input_queue_size < 1:
            raise ValueError('input_queue_size must be positive')
        if self.ws_queue_size < 1:
            raise ValueError('ws_queue_size must be positive')
        if worker_threads < 1:
            raise ValueError('worker_threads must be positive')

        self.loop = asyncio.get_running_loop()
        self.streams: dict[str, _StreamState] = {}
        self._ws_send_locks: dict[Any, asyncio.Lock] = {}
        self._fallback_wire_seq = 0
        self._closed = False
        self._executor = ThreadPoolExecutor(
            max_workers=worker_threads,
            thread_name_prefix='mpegts-fanout',
        )

        # aiohttp invokes this during graceful shutdown. The getattr keeps the
        # handler usable with the lightweight ServerBase fakes used by tests.
        app = getattr(self.srv, 'app', None)
        if app is not None:
            app.on_cleanup.append(self._on_cleanup)

    @utils.handle_exceptions
    async def get_names_handler(self, request):
        topic_name_type_list = ros_handle.get_topic_names_and_types()
        vs_topics = [
            topic_name
            for topic_name, topic_types in topic_name_type_list
            if 'mpeg_ts_image_transport_msgs/msg/MpegTsDatagram' in topic_types
        ]

        return web.Response(text=json.dumps({
            'success': True,
            'message': 'ok',
            'topics': vs_topics,
        }))

    @staticmethod
    def _encode_datagram(
        stream_name: str,
        ros_seq: int,
        data: bytes,
        first_wire_seq: int,
    ) -> tuple[bytes, ...]:
        """Validate, split, and serialize a ROS datagram in a worker thread."""
        if len(data) % 188 != 0:
            raise ValueError(
                f'received {len(data)} bytes for stream {stream_name}, '
                'which is not a multiple of 188'
            )

        encoded_packets: list[bytes] = []
        for packet_index, offset in enumerate(range(0, len(data), 188)):
            ts_packet = data[offset:offset + 188]
            if ts_packet[0] != 0x47:
                raise ValueError(
                    f'received TS packet with invalid sync byte for stream {stream_name}'
                )

            message = generic_pb2.Message()
            message.seq = first_wire_seq + packet_index
            message.mpeg_ts_datagram.stream_name = stream_name
            message.mpeg_ts_datagram.data = ts_packet
            message.mpeg_ts_datagram.seq = ros_seq
            encoded_packets.append(message.SerializeToString())

        return tuple(encoded_packets)

    def _reserve_wire_sequences(self, count: int) -> int:
        """Reserve sequence IDs without serializing on the asyncio thread."""
        if hasattr(self.srv, 'udp_msg_seq'):
            first = self.srv.udp_msg_seq
            self.srv.udp_msg_seq += count
            return first

        first = self._fallback_wire_seq
        self._fallback_wire_seq += count
        return first

    @staticmethod
    def _put_latest(queue: asyncio.Queue, item) -> None:
        """Put without blocking, dropping the oldest queued item if necessary."""
        if queue.full():
            try:
                queue.get_nowait()
            except asyncio.QueueEmpty:
                pass

        try:
            queue.put_nowait(item)
        except asyncio.QueueFull:
            # Another producer can win between get_nowait() and put_nowait().
            pass

    def _enqueue_ros_datagram(
        self,
        stream_name: str,
        state: _StreamState,
        item: tuple[int, bytes],
    ) -> None:
        if self._closed or self.streams.get(stream_name) is not state:
            return
        self._put_latest(state.input_queue, item)

    def on_th_pkt_recv(
        self,
        msg: MpegTsDatagram,
        stream_name: str,
        state: _StreamState,
    ) -> None:
        """ROS callback: copy data and return; all processing happens elsewhere."""
        item = (msg.seq, bytes(msg.data))
        self.loop.call_soon_threadsafe(
            self._enqueue_ros_datagram,
            stream_name,
            state,
            item,
        )

    def _send_udp_packet(self, payload: bytes, state: _StreamState) -> None:
        """Send pre-serialized video bytes on the established UDP socket."""
        udp_socket = getattr(self.srv, 'udp', None)
        live_udp_clients = getattr(self.srv, 'udp_clients', set())

        expired = {client for client in state.udp_clients if client not in live_udp_clients}
        state.udp_clients.difference_update(expired)

        if udp_socket is None:
            return

        for client in tuple(state.udp_clients):
            try:
                udp_socket.sendto(payload, client)
            except (ConnectionError, OSError, RuntimeError) as exc:
                print(f'removing failed udp client {client}: {exc}')
                state.udp_clients.discard(client)

    async def _ws_writer_loop(
        self,
        stream_name: str,
        state: _StreamState,
        websocket,
        writer: _WebSocketWriter,
    ) -> None:
        try:
            while self.streams.get(stream_name) is state:
                payload = await writer.queue.get()

                live_websockets = getattr(self.srv, 'ws_clients', None)
                if live_websockets is not None and websocket not in live_websockets:
                    break

                async with writer.send_lock:
                    await asyncio.wait_for(
                        websocket.send_bytes(payload),
                        timeout=self.send_timeout_sec,
                    )
        except asyncio.CancelledError:
            raise
        except (asyncio.TimeoutError, ConnectionError, OSError, RuntimeError) as exc:
            print(f'removing slow/failed websocket from stream {stream_name}: {exc}')
        except BaseException as exc:
            print(f'error sending MPEG-TS stream {stream_name}: {exc}')
        finally:
            current = state.ws_clients.get(websocket)
            if current is writer:
                del state.ws_clients[websocket]
            self._maybe_stop_stream(stream_name, state)

    def _add_ws_client(self, stream_name: str, state: _StreamState, websocket) -> None:
        if websocket in state.ws_clients:
            return

        send_lock = self._ws_send_locks.setdefault(websocket, asyncio.Lock())
        writer = _WebSocketWriter(
            queue=asyncio.Queue(maxsize=self.ws_queue_size),
            send_lock=send_lock,
        )
        state.ws_clients[websocket] = writer
        writer.task = self.srv.schedule_task(
            self._ws_writer_loop(stream_name, state, websocket, writer)
        )

    def _fan_out_packet(self, payload: bytes, state: _StreamState) -> None:
        # UDP sendto() is non-blocking, so a second UDP source socket would not
        # remove meaningful work from the event loop. Reusing the discovered
        # socket also avoids changing source-port/NAT behaviour for clients.
        self._send_udp_packet(payload, state)

        # WebSocket writes remain on the asyncio loop, but every client has an
        # independent bounded queue and writer task. A slow peer can therefore
        # drop its own stale video packets without delaying any other peer.
        for writer in tuple(state.ws_clients.values()):
            self._put_latest(writer.queue, payload)

    async def _run_stream(self, stream_name: str, state: _StreamState) -> None:
        print(f'{stream_name} started')

        try:
            while self.streams.get(stream_name) is state:
                ros_seq, data = await state.input_queue.get()

                if state.last_ros_seq != -1 and ros_seq > state.last_ros_seq + 1:
                    skipped = ros_seq - state.last_ros_seq - 1
                    print(f'WARNING: skipped {skipped} datagrams for stream {stream_name}')
                state.last_ros_seq = ros_seq

                if len(data) % 188 != 0:
                    print(
                        f'WARNING: received {len(data)} bytes for stream {stream_name}, '
                        'which is not a multiple of 188'
                    )
                    continue

                packet_count = len(data) // 188
                first_wire_seq = self._reserve_wire_sequences(packet_count)

                try:
                    encode = functools.partial(
                        self._encode_datagram,
                        stream_name,
                        ros_seq,
                        data,
                        first_wire_seq,
                    )
                    packets = await self.loop.run_in_executor(self._executor, encode)
                except ValueError as exc:
                    print(f'WARNING: {exc}')
                    continue

                for packet_index, payload in enumerate(packets, start=1):
                    if self.streams.get(stream_name) is not state:
                        break

                    self._fan_out_packet(payload, state)

                    if (
                        self.event_loop_yield_packets > 0
                        and packet_index % self.event_loop_yield_packets == 0
                    ):
                        await asyncio.sleep(0)

                self._maybe_stop_stream(stream_name, state)
        except asyncio.CancelledError:
            raise
        finally:
            if self.streams.get(stream_name) is state:
                self._remove_stream(stream_name, state)
            print(f'{stream_name} exiting')

    def _remove_stream(self, stream_name: str, state: _StreamState) -> None:
        if self.streams.get(stream_name) is state:
            del self.streams[stream_name]

        if state.subscription is not None:
            try:
                ros_handle.destroy_subscription(state.subscription)
            except BaseException as exc:
                print(f'error destroying subscription for {stream_name}: {exc}')
            state.subscription = None

        current_task = asyncio.current_task()

        if state.task is not None and state.task is not current_task:
            state.task.cancel()

        for writer in tuple(state.ws_clients.values()):
            if writer.task is not None and writer.task is not current_task:
                writer.task.cancel()
        state.ws_clients.clear()
        state.udp_clients.clear()

    def _maybe_stop_stream(self, stream_name: str, state: _StreamState) -> None:
        if state.ws_clients or state.udp_clients:
            return
        if self.streams.get(stream_name) is state:
            print(f'no more clients for stream {stream_name}, unsubscribing from ros topic')
            self._remove_stream(stream_name, state)

    def subscribe_to_stream(self, stream_name: str, proto: str, sock) -> bool:
        state = self.streams.get(stream_name)

        if state is None:
            state = _StreamState(
                input_queue=asyncio.Queue(maxsize=self.input_queue_size),
            )
            self.streams[stream_name] = state

        if proto == 'ws':
            self._add_ws_client(stream_name, state, sock)
        elif proto == 'udp':
            state.udp_clients.add(sock)
        else:
            raise ValueError(f'unsupported video transport {proto!r}')

        if state.subscription is not None:
            return True

        try:
            state.subscription = ros_handle.create_subscription(
                MpegTsDatagram,
                stream_name,
                functools.partial(
                    self.on_th_pkt_recv,
                    stream_name=stream_name,
                    state=state,
                ),
                1024,
                best_effort=True,
            )
            state.task = self.srv.schedule_task(self._run_stream(stream_name, state))
        except BaseException:
            self._remove_stream(stream_name, state)
            raise

        return True

    async def handle_ws_msg(self, msg, proto, sock):
        if msg.get('type') != 'video_request':
            return

        stream_name = msg['stream_name']
        operation = msg.get('operation', 'connect')

        if operation == 'disconnect':
            state = self.streams.get(stream_name)
            if state is not None:
                writer = state.ws_clients.pop(sock, None)
                if writer is not None and writer.task is not None:
                    writer.task.cancel()
                state.udp_clients.discard(sock)
                self._maybe_stop_stream(stream_name, state)
            await self.srv.log(f'disconnected client from stream {stream_name}')
            return

        self.subscribe_to_stream(stream_name, proto, sock)
        await self.srv.log(f'new client {proto} {sock} for stream {stream_name}')

    async def _on_cleanup(self, app) -> None:
        self._closed = True
        tasks: list[asyncio.Task] = []

        for stream_name, state in tuple(self.streams.items()):
            if state.task is not None:
                tasks.append(state.task)
            tasks.extend(
                writer.task
                for writer in state.ws_clients.values()
                if writer.task is not None
            )
            self._remove_stream(stream_name, state)

        if tasks:
            await asyncio.gather(*tasks, return_exceptions=True)

        self._executor.shutdown(wait=False, cancel_futures=True)
