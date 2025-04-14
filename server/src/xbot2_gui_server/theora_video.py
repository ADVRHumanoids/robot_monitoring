import asyncio
from aiohttp import web
import json
import base64

import rospy
from theora_image_transport.msg import Packet as TheoraPacket

from .server import ServerBase
from . import utils

from .proto import generic_pb2

class TheoraVideoHandler:

    def __init__(self, srv: ServerBase, config=dict()) -> None:

        # save server object, register our handlers
        self.srv = srv
        self.srv.register_ws_coroutine(self.handle_ws_msg)
        self.srv.add_route('GET', '/video/get_names', self.get_names_handler, 'video_get_names')
        self.srv.add_route('PUT', '/video/set_stream', self.set_stream_handler, 'video_set_stream')

        # queue for th packets
        self.th_pkt_queue = asyncio.Queue(maxsize=0)
        self.th_hdr_pkt = []

        # subscribers
        self.img_data = dict()
        self.img_sub = None
        self.img_msg_to_send = {}
        
        # clients
        self.clients = dict()

        # event loop
        self.loop = asyncio.get_event_loop()

    
    @utils.handle_exceptions
    async def get_names_handler(self, request):

        # get topic names from ros master
        topic_name_type_list = await utils.to_thread(rospy.get_published_topics)

        # filter those with theora type and nice name
        vs_topics = list()
        for tname, ttype in topic_name_type_list:
            if 'image_raw' not in tname:
                continue
            if ttype == 'theora_image_transport/Packet':
                vs_topics.append(tname)
        
        # return
        return web.Response(text=json.dumps({
            'success': True,
            'message': 'ok',
            'topics': vs_topics,
            }))

    
    @utils.handle_exceptions
    async def set_stream_handler(self, request):

        body = await request.text()
        body = json.loads(body)
        stream_name = body['stream_name']

        if len(stream_name) == 0:
            return

        if stream_name in self.img_data.keys():
            img_sub = self.img_data[stream_name][1]
            img_sub.unregister()

        
        img_hdr_pkt = list()

        img_msg_queue = asyncio.Queue(maxsize=0)

        self.img_data[stream_name] = [stream_name, None, img_hdr_pkt, img_msg_queue]
        
        print(f'requested stream {stream_name}, registering subscriber and waiting for headers..')

        img_sub = rospy.Subscriber(stream_name, 
            TheoraPacket, self.on_th_pkt_recv, self.img_data[stream_name], queue_size=20, tcp_nodelay=True)

        self.img_data[stream_name][1] = img_sub

        # wait for headers
        iter = 0
        while len(img_hdr_pkt) < 3 and iter < 500:
            await asyncio.sleep(0.1)

        if len(img_hdr_pkt) < 3:
            return web.Response(text=json.dumps({
                'success': False,
                'message': f'failed to receive headers for "{stream_name}"',
                }))

        print('got headers')

        self.srv.schedule_task(self.run(self.img_data[stream_name]))

        return web.Response(text=json.dumps({
                'success': True,
                'message': f'subscribed to "{stream_name}"',
                'hdr': img_hdr_pkt,
                }))


    
    async def run(self, stream_data):

        # unpack
        stream_name, img_sub, img_hdr_pkt, img_msg_queue = stream_data
        
        # check if we have a client for this stream
        if stream_name not in self.clients.keys():
            self.clients[stream_name] = {'ws': set(), 'udp': set()}

        print(f'{stream_name} started')

        while img_sub is not None:

            # await for a new packet to be received from ros
            th_pkt = await img_msg_queue.get()

            # iterate over sockets (one per client)
            ws_clients = self.clients[stream_name]['ws']
            udp_clients = self.clients[stream_name]['udp']
            expired_udp = await self.srv.udp_send_to_all(msg=th_pkt, clients=udp_clients)
            expired_ws = await self.srv.ws_send_to_all(msg=th_pkt, clients=ws_clients)

        print(f'{stream_name} exiting')

    
    def on_th_pkt_recv(self, msg: TheoraPacket, stream_data):

        stream_name, img_sub, img_hdr_pkt, img_msg_queue = stream_data
        
        pbmsg = generic_pb2.Message()
        pbmsg.theora_packet.stream_name = stream_name
        pbmsg.theora_packet.data = msg.data
        pbmsg.theora_packet.b_o_s = msg.b_o_s
        pbmsg.theora_packet.e_o_s = msg.e_o_s
        pbmsg.theora_packet.granulepos = msg.granulepos
        pbmsg.theora_packet.packetno = msg.packetno
        
        def json_msg():
            th_pkt = dict()
            th_pkt['type'] = 'theora'
            th_pkt['streamName'] = stream_name
            th_pkt['data'] = base64.b64encode(msg.data).decode('ascii')
            th_pkt['bOS'] = msg.b_o_s
            th_pkt['eOS'] = msg.e_o_s
            th_pkt['granulepos'] = msg.granulepos
            th_pkt['packetno'] = msg.packetno
            return th_pkt

        if msg.b_o_s == 1:
            img_hdr_pkt.append(json_msg())
            print(f'got header {len(img_hdr_pkt)}/3')
            return 

        if msg.granulepos == 0:
            img_hdr_pkt.append(json_msg())
            print(f'got header {len(img_hdr_pkt)}/3')
            return

        _ = asyncio.run_coroutine_threadsafe(img_msg_queue.put(pbmsg), self.loop)
        
        
    async def handle_ws_msg(self, msg, proto, sock):
        if msg['type'] == 'video_request':
            stream_name = msg['stream_name']
            op = msg.get('operation', 'connect')
            if stream_name not in self.clients.keys():
                self.clients[stream_name] = {'ws': set(), 'udp': set()}
            if op == 'disconnect':
                self.clients[stream_name]['ws'].discard(sock)
                self.clients[stream_name]['udp'].discard(sock)
                await self.srv.log(f'disconnected client from stream {stream_name}')
            else:
                self.clients[stream_name][proto].add(sock)
                await self.srv.log(f'new client {proto} {sock} for stream {stream_name}')
            
            print(self.clients)
