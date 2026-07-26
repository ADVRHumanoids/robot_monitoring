import asyncio
import functools
import multiprocessing
import os
import queue
import socket
import time
import traceback
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
class _WorkerStream:
    subscription: Any
    udp_clients: set[Any] = field(default_factory=set)
    last_ros_seq: int = -1


def _serialize_datagram(
    stream_name: str,
    ros_seq: int,
    data: bytes,
    first_wire_seq: int,
) -> tuple[tuple[bytes, ...], int]:
    """Validate, split, and serialize one MPEG-TS ROS datagram."""
    if len(data) % 188 != 0:
        raise ValueError(
            f'received {len(data)} bytes for stream {stream_name}, '
            'which is not a multiple of 188'
        )

    payloads: list[bytes] = []
    wire_seq = first_wire_seq

    for offset in range(0, len(data), 188):
        ts_packet = data[offset:offset + 188]
        if not ts_packet or ts_packet[0] != 0x47:
            raise ValueError(
                f'received TS packet with invalid sync byte for stream {stream_name}'
            )

        message = generic_pb2.Message()
        message.seq = wire_seq
        message.mpeg_ts_datagram.stream_name = stream_name
        message.mpeg_ts_datagram.data = ts_packet
        message.mpeg_ts_datagram.seq = ros_seq
        payloads.append(message.SerializeToString())
        wire_seq = (wire_seq + 1) & 0x7FFFFFFF

    return tuple(payloads), wire_seq


class _MpegTsProcessRuntime:
    """ROS 2 executor and UDP fan-out owned entirely by the worker process."""

    def __init__(self, config: dict, commands) -> None:
        self._config = config
        self._commands = commands
        self._stop_requested = False
        self._node = None
        self._executor = None
        self._udp_socket: socket.socket | None = None
        self._streams: dict[str, _WorkerStream] = {}
        self._wire_seq = 0
        self._dropped_udp_packets = 0
        self._last_udp_drop_log = 0.0

    def run(self, ready_connection) -> None:
        import rclpy
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.node import Node

        try:
            rclpy.init(signal_handler_options=rclpy.SignalHandlerOptions.NO)
            self._node = Node('xbot2_gui_mpegts')
            self._executor = SingleThreadedExecutor(context=self._node.context)
            self._executor.add_node(self._node)

            self._udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            send_buffer = int(self._config['udp_send_buffer_bytes'])
            if send_buffer > 0:
                self._udp_socket.setsockopt(
                    socket.SOL_SOCKET,
                    socket.SO_SNDBUF,
                    send_buffer,
                )
            self._udp_socket.bind(
                (str(self._config['udp_host']), int(self._config['udp_port']))
            )
            self._udp_socket.setblocking(False)
            udp_port = self._udp_socket.getsockname()[1]

            ready_connection.send({
                'ok': True,
                'pid': os.getpid(),
                'udp_port': udp_port,
            })
            ready_connection.close()

            print(
                f'MPEG-TS process {os.getpid()} started with dedicated ROS 2 '
                f'executor and UDP source port {udp_port}'
            )

            spin_timeout_sec = float(self._config['spin_timeout_sec'])
            while not self._stop_requested:
                self._drain_commands()
                if self._stop_requested:
                    break
                self._executor.spin_once(timeout_sec=spin_timeout_sec)

            self._drain_commands()
        except BaseException as exc:
            try:
                ready_connection.send({
                    'ok': False,
                    'error': repr(exc),
                    'traceback': traceback.format_exc(),
                })
                ready_connection.close()
            except BaseException:
                pass
            traceback.print_exc()
        finally:
            self._destroy_all_streams()

            if self._executor is not None:
                try:
                    self._executor.shutdown(timeout_sec=1.0)
                except BaseException:
                    traceback.print_exc()

            if self._node is not None:
                try:
                    self._node.destroy_node()
                except BaseException:
                    traceback.print_exc()

            if self._udp_socket is not None:
                self._udp_socket.close()

            try:
                import rclpy
                if rclpy.ok():
                    rclpy.shutdown()
            except BaseException:
                traceback.print_exc()

            print(f'MPEG-TS process {os.getpid()} stopped')

    def _drain_commands(self) -> None:
        while True:
            try:
                operation, *args = self._commands.get_nowait()
            except queue.Empty:
                return

            if operation == 'shutdown':
                self._stop_requested = True
                return
            if operation == 'add_udp':
                self._add_udp_client(args[0], args[1])
            elif operation == 'remove_udp':
                self._remove_udp_client(args[0], args[1])
            else:
                print(f'unknown MPEG-TS process command: {operation}')

    def _create_stream(self, stream_name: str) -> _WorkerStream:
        from rclpy.qos import (
            QoSDurabilityPolicy,
            QoSHistoryPolicy,
            QoSProfile,
            QoSReliabilityPolicy,
        )

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=int(self._config['qos_depth']),
        )
        subscription = self._node.create_subscription(
            MpegTsDatagram,
            stream_name,
            functools.partial(self._on_datagram, stream_name=stream_name),
            qos_profile,
        )
        stream = _WorkerStream(subscription=subscription)
        self._streams[stream_name] = stream
        print(f'MPEG-TS process subscribed to {stream_name}')
        return stream

    def _destroy_stream(self, stream_name: str) -> None:
        stream = self._streams.pop(stream_name, None)
        if stream is None or self._node is None:
            return
        self._node.destroy_subscription(stream.subscription)
        print(f'MPEG-TS process unsubscribed from {stream_name}')

    def _destroy_all_streams(self) -> None:
        for stream_name in tuple(self._streams):
            try:
                self._destroy_stream(stream_name)
            except BaseException:
                traceback.print_exc()

    def _add_udp_client(self, stream_name: str, client) -> None:
        stream = self._streams.get(stream_name)
        if stream is None:
            stream = self._create_stream(stream_name)
        stream.udp_clients.add(client)

    def _remove_udp_client(self, stream_name: str, client) -> None:
        stream = self._streams.get(stream_name)
        if stream is None:
            return
        stream.udp_clients.discard(client)
        if not stream.udp_clients:
            self._destroy_stream(stream_name)

    def _on_datagram(self, msg: MpegTsDatagram, stream_name: str) -> None:
        """This ROS callback and all downstream work run in the child process."""
        try:
            stream = self._streams.get(stream_name)
            if stream is None or not stream.udp_clients:
                return

            ros_seq = int(msg.seq)
            if stream.last_ros_seq != -1 and ros_seq > stream.last_ros_seq + 1:
                skipped = ros_seq - stream.last_ros_seq - 1
                print(f'WARNING: skipped {skipped} MPEG-TS datagrams for {stream_name}')
            stream.last_ros_seq = ros_seq

            payloads, self._wire_seq = _serialize_datagram(
                stream_name,
                ros_seq,
                bytes(msg.data),
                self._wire_seq,
            )
            self._send_udp(payloads, stream)
        except ValueError as exc:
            print(f'WARNING: {exc}')
        except BaseException:
            traceback.print_exc()

    def _send_udp(self, payloads: tuple[bytes, ...], stream: _WorkerStream) -> None:
        if self._udp_socket is None:
            return

        for payload in payloads:
            for client in tuple(stream.udp_clients):
                try:
                    self._udp_socket.sendto(payload, client)
                except BlockingIOError:
                    self._record_udp_drop()
                except (ConnectionError, OSError, RuntimeError) as exc:
                    print(f'removing failed MPEG-TS UDP client {client}: {exc}')
                    stream.udp_clients.discard(client)

    def _record_udp_drop(self) -> None:
        self._dropped_udp_packets += 1
        now = time.monotonic()
        if now - self._last_udp_drop_log >= 1.0:
            print(
                'WARNING: MPEG-TS UDP socket dropped '
                f'{self._dropped_udp_packets} packets because its send buffer was full'
            )
            self._dropped_udp_packets = 0
            self._last_udp_drop_log = now


def _mpegts_process_main(config: dict, commands, ready_connection) -> None:
    _MpegTsProcessRuntime(config, commands).run(ready_connection)


class _MpegTsPipelineProcess:
    def __init__(self, config: dict) -> None:
        self._context = multiprocessing.get_context('spawn')
        self._commands = self._context.Queue(maxsize=int(config['command_queue_size']))
        parent_connection, child_connection = self._context.Pipe(duplex=False)
        self._parent_connection = parent_connection
        self._child_connection = child_connection
        self._process = self._context.Process(
            target=_mpegts_process_main,
            args=(config, self._commands, child_connection),
            name='mpegts-pipeline',
            daemon=True,
        )
        self.pid: int | None = None
        self.udp_port = 0

    @property
    def is_alive(self) -> bool:
        return self._process.is_alive()

    def start(self, timeout_sec: float) -> None:
        self._process.start()
        self._child_connection.close()

        if not self._parent_connection.poll(timeout_sec):
            self._parent_connection.close()
            self._process.terminate()
            self._process.join(timeout=1.0)
            self._close_queue()
            raise TimeoutError('timed out starting the MPEG-TS pipeline process')

        try:
            status = self._parent_connection.recv()
        except EOFError as exc:
            self._parent_connection.close()
            self._process.join(timeout=1.0)
            self._close_queue()
            raise RuntimeError('MPEG-TS pipeline process exited during startup') from exc
        self._parent_connection.close()
        if not status.get('ok'):
            self._process.join(timeout=1.0)
            self._close_queue()
            details = status.get('traceback') or status.get('error') or 'unknown error'
            raise RuntimeError(f'failed to start MPEG-TS pipeline process:\n{details}')

        self.pid = int(status['pid'])
        self.udp_port = int(status['udp_port'])

    def submit(self, operation: str, *args) -> None:
        if not self._process.is_alive():
            raise RuntimeError('MPEG-TS pipeline process is not running')
        try:
            self._commands.put_nowait((operation, *args))
        except queue.Full as exc:
            raise RuntimeError('MPEG-TS pipeline command queue is full') from exc

    def _close_queue(self) -> None:
        self._commands.close()
        self._commands.join_thread()

    def close(self, timeout_sec: float = 5.0) -> None:
        if not self._process.is_alive():
            self._process.join(timeout=0.1)
            self._close_queue()
            return

        try:
            self.submit('shutdown')
        except RuntimeError:
            pass

        self._process.join(timeout=timeout_sec)
        if self._process.is_alive():
            self._process.terminate()
            self._process.join(timeout=1.0)
            if self._process.is_alive():
                self._process.kill()
                self._process.join(timeout=1.0)

        self._close_queue()


class MpegTsVideoHandler:

    def __init__(self, srv: ServerBase, config: dict | None = None) -> None:
        config = config or {}

        if getattr(ros_handle, 'ros_version', None) != 2:
            raise RuntimeError('the isolated MPEG-TS pipeline requires ROS 2')

        self.srv = srv
        self._closed = False
        self.srv.register_ws_coroutine(self.handle_ws_msg)
        self.srv.add_route('GET', '/video/get_names', self.get_names_handler, 'video_get_names')
        self.srv.add_route(
            'GET',
            '/video/transport',
            self.get_transport_handler,
            'video_transport',
        )

        process_config = {
            'udp_host': str(config.get('udp_host', '0.0.0.0')),
            'udp_port': int(config.get('udp_port', 0)),
            'udp_send_buffer_bytes': int(
                config.get('udp_send_buffer_bytes', 4 * 1024 * 1024)
            ),
            'qos_depth': int(config.get('qos_depth', 8)),
            'spin_timeout_sec': float(config.get('spin_timeout_sec', 0.005)),
            'command_queue_size': int(config.get('command_queue_size', 256)),
        }
        if process_config['qos_depth'] < 1:
            raise ValueError('qos_depth must be positive')
        if process_config['command_queue_size'] < 1:
            raise ValueError('command_queue_size must be positive')

        self._pipeline = _MpegTsPipelineProcess(process_config)
        self._pipeline.start(float(config.get('startup_timeout_sec', 10.0)))

        app = getattr(self.srv, 'app', None)
        if app is not None:
            app.on_cleanup.append(self._on_cleanup)

    @utils.handle_exceptions
    async def get_names_handler(self, request):
        topic_name_type_list = ros_handle.get_topic_names_and_types()
        topics = [
            topic_name
            for topic_name, topic_types in topic_name_type_list
            if 'mpeg_ts_image_transport_msgs/msg/MpegTsDatagram' in topic_types
        ]
        return web.json_response({
            'success': True,
            'message': 'ok',
            'topics': topics,
        })

    @utils.handle_exceptions
    async def get_transport_handler(self, request):
        return web.json_response({
            'success': True,
            'message': 'MPEG-TS runs in an isolated ROS 2 process',
            'mode': 'dedicated_process_udp',
            'pid': self._pipeline.pid,
            'udp_source_port': self._pipeline.udp_port,
            'websocket_video_supported': False,
            'alive': self._pipeline.is_alive,
        })

    def subscribe_to_stream(self, stream_name: str, proto: str, sock) -> bool:
        if proto != 'udp':
            raise RuntimeError(
                'isolated MPEG-TS transport is UDP-only; websocket video would '
                'put frame delivery back on the main asyncio loop'
            )
        self._pipeline.submit('add_udp', stream_name, sock)
        return True

    async def handle_ws_msg(self, msg, proto, sock):
        if msg.get('type') != 'video_request':
            return

        stream_name = msg['stream_name']
        operation = msg.get('operation', 'connect')

        if proto != 'udp':
            await self.srv.log(
                'MPEG-TS websocket transport is disabled because the video data '
                'plane is isolated from the main asyncio loop',
                sev=1,
            )
            return

        if operation == 'disconnect':
            self._pipeline.submit('remove_udp', stream_name, sock)
            await self.srv.log(f'disconnected UDP client from stream {stream_name}')
            return

        self.subscribe_to_stream(stream_name, proto, sock)
        await self.srv.log(
            f'new MPEG-TS UDP client {sock} for stream {stream_name}; '
            f'worker pid={self._pipeline.pid}'
        )

    async def _on_cleanup(self, app) -> None:
        if self._closed:
            return
        self._closed = True
        await asyncio.to_thread(self._pipeline.close)
