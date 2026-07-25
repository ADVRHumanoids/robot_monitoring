import asyncio
import functools
import json

from aiohttp import web

# ros handle
from . import ros_utils
from . import utils
from .proto import generic_pb2
from .server import ServerBase

ros_handle: ros_utils.RosWrapper = ros_utils.ros_handle

from mpeg_ts_image_transport_msgs.msg import MpegTsDatagram


class MpegTsVideoHandler:

    def __init__(self, srv: ServerBase, config=dict()) -> None:

        # save server object, register our handlers
        self.srv = srv
        self.srv.register_ws_coroutine(self.handle_ws_msg)
        self.srv.add_route('GET', '/video/get_names', self.get_names_handler, 'video_get_names')

        # A ROS callback must never perform packet splitting, protobuf creation, or
        # network fan-out. It only copies the incoming datagram into this bounded
        # queue. The consumer moves CPU work to asyncio's worker thread.
        self.input_queue_size = int(config.get('input_queue_size', 8))
        self.send_timeout_sec = float(config.get('send_timeout_sec', 0.25))
        self.queue_dict: dict[str, asyncio.Queue] = dict()

        # clients
        self.clients = dict()

        # event loop
        self.loop = asyncio.get_event_loop()

    @utils.handle_exceptions
    async def get_names_handler(self, request):

        # get topic names from ros master
        topic_name_type_list = ros_handle.get_topic_names_and_types()
        print(topic_name_type_list)

        # filter those with mpeg_ts type and nice name
        vs_topics = list()
        for tname, ttypes in topic_name_type_list:
            if 'mpeg_ts_image_transport_msgs/msg/MpegTsDatagram' in ttypes:
                vs_topics.append(tname)

        # return
        return web.Response(text=json.dumps({
            'success': True,
            'message': 'ok',
            'topics': vs_topics,
        }))

    @staticmethod
    def _encode_datagram(stream_name: str, seq: int, data: bytes):
        """Split and encode one ROS datagram outside the asyncio event loop."""
        if len(data) % 188 != 0:
            raise ValueError(
                f'received {len(data)} bytes for stream {stream_name}, '
                'which is not a multiple of 188'
            )

        encoded_packets = []
        for offset in range(0, len(data), 188):
            ts_pkt = data[offset:offset + 188]
            if ts_pkt[0] != 0x47:
                raise ValueError(
                    f'received TS packet with invalid sync byte for stream {stream_name}'
                )

            pbmsg = generic_pb2.Message()
            pbmsg.mpeg_ts_datagram.stream_name = stream_name
            pbmsg.mpeg_ts_datagram.data = ts_pkt
            pbmsg.mpeg_ts_datagram.seq = seq
            encoded_packets.append(pbmsg)

        return encoded_packets

    async def _send_one(self, method, msg, client):
        """Send to one client without allowing it to block the whole stream."""
        try:
            expired = await asyncio.wait_for(
                method(msg=msg, clients={client}),
                timeout=self.send_timeout_sec,
            )
            return client in expired
        except (asyncio.TimeoutError, ConnectionError, RuntimeError):
            return True
        except BaseException as exc:
            print(f'error sending MPEG-TS packet to {client}: {exc}')
            return True

    async def _fan_out(self, stream_name: str, msg):
        """Fan out concurrently while keeping all socket I/O on the event loop."""
        stream_clients = self.clients.get(stream_name)
        if stream_clients is None:
            return

        udp_clients = tuple(stream_clients['udp'])
        ws_clients = tuple(stream_clients['ws'])

        udp_results, ws_results = await asyncio.gather(
            asyncio.gather(*(
                self._send_one(self.srv.udp_send_to_all, msg, client)
                for client in udp_clients
            )),
            asyncio.gather(*(
                self._send_one(self.srv.ws_send_to_all, msg, client)
                for client in ws_clients
            )),
        )

        for client, expired in zip(udp_clients, udp_results):
            if expired:
                print(f'removing expired udp client {client} from stream {stream_name}')
                stream_clients['udp'].discard(client)

        for client, expired in zip(ws_clients, ws_results):
            if expired:
                print(f'removing expired ws client {client} from stream {stream_name}')
                stream_clients['ws'].discard(client)

    async def run(self, stream_data):

        # unpack
        stream_name, sub, queue = stream_data

        print(f'{stream_name} started')

        last_seq = -1

        while stream_name in self.queue_dict:

            # Wait for a complete ROS datagram. The ROS callback only copies and
            # enqueues it, so the rclpy/asyncio loop remains responsive.
            seq, data = await queue.get()

            if last_seq != -1 and seq > last_seq + 1:
                skipped = seq - last_seq - 1
                print(f'WARNING: skipped {skipped} datagrams for stream {stream_name}')
            last_seq = seq

            try:
                encode = functools.partial(
                    self._encode_datagram,
                    stream_name,
                    seq,
                    data,
                )
                packets = await self.loop.run_in_executor(None, encode)
            except ValueError as exc:
                print(f'WARNING: {exc}')
                continue

            # Socket objects belong to aiohttp/asyncio and must stay on this loop.
            # Each client is nevertheless awaited concurrently and has a timeout,
            # so one slow peer cannot stall all other peers.
            for packet in packets:
                await self._fan_out(stream_name, packet)

                stream_clients = self.clients.get(stream_name)
                if stream_clients is None:
                    break

                if not stream_clients['ws'] and not stream_clients['udp']:
                    print(f'no more clients for stream {stream_name}, unsubscribing from ros topic')
                    ros_handle.destroy_subscription(sub)
                    del self.queue_dict[stream_name]
                    del self.clients[stream_name]
                    break

        print(f'{stream_name} exiting')

    def on_th_pkt_recv(self, msg: MpegTsDatagram, stream_name: str):
        """Minimal ROS callback with bounded, drop-oldest backpressure."""
        queue = self.queue_dict.get(stream_name)
        if queue is None:
            return

        item = (msg.seq, bytes(msg.data))

        if queue.full():
            try:
                queue.get_nowait()
            except asyncio.QueueEmpty:
                pass

        try:
            queue.put_nowait(item)
        except asyncio.QueueFull:
            # A producer race can fill the queue between get_nowait and put_nowait.
            # Dropping video is preferable to blocking ROS and the HTTP event loop.
            pass

    def subscribe_to_stream(self, stream_name: str, proto: str, sock: str):

        # check if we have a client for this stream
        if stream_name not in self.clients:
            self.clients[stream_name] = {'ws': set(), 'udp': set()}

        # add the client to the list
        self.clients[stream_name][proto].add(sock)

        # check if we already have a subscriber for this stream
        if stream_name in self.queue_dict:
            return True

        # create a bounded queue for this stream
        img_msg_queue = asyncio.Queue(maxsize=self.input_queue_size)
        self.queue_dict[stream_name] = img_msg_queue

        # subscribe to the ros topic
        img_sub = ros_handle.create_subscription(
            MpegTsDatagram,
            stream_name,
            functools.partial(self.on_th_pkt_recv, stream_name=stream_name),
            1024,
            best_effort=True,
        )

        # start the run loop
        self.srv.schedule_task(self.run((stream_name, img_sub, img_msg_queue)))

        return True

    async def handle_ws_msg(self, msg, proto, sock):
        if msg['type'] == 'video_request':
            stream_name = msg['stream_name']
            op = msg.get('operation', 'connect')
            if op == 'disconnect':
                stream_clients = self.clients.get(stream_name)
                if stream_clients is not None:
                    stream_clients['ws'].discard(sock)
                    stream_clients['udp'].discard(sock)
                await self.srv.log(f'disconnected client from stream {stream_name}')
            else:
                self.subscribe_to_stream(stream_name, proto, sock)
                await self.srv.log(f'new client {proto} {sock} for stream {stream_name}')
