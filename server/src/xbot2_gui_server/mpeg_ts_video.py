import asyncio
from aiohttp import web
import json
import base64
import functools

# ros handle
from . import ros_utils
ros_handle : ros_utils.RosWrapper = ros_utils.ros_handle

from mpeg_ts_image_transport_msgs.msg import MpegTsDatagram
from .server import ServerBase
from . import utils

from .proto import generic_pb2

class MpegTsVideoHandler:

    def __init__(self, srv: ServerBase, config=dict()) -> None:

        # save server object, register our handlers
        self.srv = srv
        self.srv.register_ws_coroutine(self.handle_ws_msg)
        self.srv.add_route('GET', '/video/get_names', self.get_names_handler, 'video_get_names')

        # subscribers
        self.queue_dict : dict[str, asyncio.Queue] = dict()
        
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
            # if 'color' not in tname:
            #     continue
            if 'mpeg_ts_image_transport_msgs/msg/MpegTsDatagram' in ttypes:
                vs_topics.append(tname)
        
        # return
        return web.Response(text=json.dumps({
            'success': True,
            'message': 'ok',
            'topics': vs_topics,
            }))

    
    async def run(self, stream_data):

        # unpack
        stream_name, sub, queue = stream_data
        
        print(f'{stream_name} started')
        
        last_seq = -1

        while stream_name in self.queue_dict.keys():

            # await for a new packet to be received from ros
            th_pkt = await queue.get()
            
            # check for sequence number
            if last_seq != -1 and th_pkt.mpeg_ts_datagram.seq > last_seq + 1:
                skipped = th_pkt.mpeg_ts_datagram.seq - last_seq - 1
                print(f'WARNING: skipped {skipped} packets for stream {stream_name}')
            last_seq = th_pkt.mpeg_ts_datagram.seq

            # iterate over sockets (one per client)
            ws_clients = self.clients[stream_name]['ws']
            udp_clients = self.clients[stream_name]['udp']
            expired_udp = await self.srv.udp_send_to_all(msg=th_pkt, clients=udp_clients)
            expired_ws = await self.srv.ws_send_to_all(msg=th_pkt, clients=ws_clients)
                        
            # remove expired clients
            for c in expired_udp:
                print(f'removing expired udp client {c} from stream {stream_name}')
                self.clients[stream_name]['udp'].discard(c)
                
            for c in expired_ws:
                print(f'removing expired ws client {c} from stream {stream_name}')
                self.clients[stream_name]['ws'].discard(c)
                
            # if there are no more clients, unsubscribe from the ros topic and remove the queue
            if len(self.clients[stream_name]['ws']) == 0 and len(self.clients[stream_name]['udp']) == 0:
                print(f'no more clients for stream {stream_name}, unsubscribing from ros topic')
                ros_handle.destroy_subscription(sub)
                del self.queue_dict[stream_name]
                del self.clients[stream_name]
                break

        print(f'{stream_name} exiting')

    
    def on_th_pkt_recv(self, msg: MpegTsDatagram, stream_name: str):
        
        # check we received a full number of TS packets
        if len(msg.data) % 188 != 0:
            print(f'WARNING: received {len(msg.data)} bytes for stream {stream_name}, which is not a multiple of 188')
            return
        
        # enque each TS packet in the queue
        queue = self.queue_dict[stream_name]
        n_ts_pkts = len(msg.data) // 188
        for i in range(n_ts_pkts):
           
            ts_pkt = msg.data[i*188:(i+1)*188]
            if ts_pkt[0] != 0x47:
                print(f'WARNING: received TS packet with invalid sync byte for stream {stream_name}')
                return
           
            # fill protobuf message        
            pbmsg = generic_pb2.Message()
            pbmsg.mpeg_ts_datagram.stream_name = stream_name
            pbmsg.mpeg_ts_datagram.data = ts_pkt.tobytes()
            pbmsg.mpeg_ts_datagram.seq = msg.seq

            # queue the message
            queue.put_nowait(pbmsg)
        

    def subscribe_to_stream(self, stream_name: str, proto: str, sock: str):

        # check if we have a client for this stream
        if stream_name not in self.clients.keys():
            self.clients[stream_name] = {'ws': set(), 'udp': set()}
        
        # add the client to the list
        self.clients[stream_name][proto].add(sock)

        # check if we already have a subscriber for this stream
        # if we do, just return
        if stream_name in self.queue_dict.keys():
            return True

        # create a queue for this stream
        img_msg_queue = asyncio.Queue()
        self.queue_dict[stream_name] = img_msg_queue

        # subscribe to the ros topic
        img_sub = ros_handle.create_subscription(
            MpegTsDatagram,
            stream_name, 
            functools.partial(self.on_th_pkt_recv, stream_name=stream_name),
            1024,
            best_effort=True)

        # start the run loop
        self.srv.schedule_task(self.run((stream_name, img_sub, img_msg_queue)))

        return True
        
        
    async def handle_ws_msg(self, msg, proto, sock):
        if msg['type'] == 'video_request':
            stream_name = msg['stream_name']
            op = msg.get('operation', 'connect')
            if op == 'disconnect':
                self.clients[stream_name]['ws'].discard(sock)
                self.clients[stream_name]['udp'].discard(sock)
                await self.srv.log(f'disconnected client from stream {stream_name}')
            else:
                self.subscribe_to_stream(stream_name, proto, sock)
                await self.srv.log(f'new client {proto} {sock} for stream {stream_name}')
            
