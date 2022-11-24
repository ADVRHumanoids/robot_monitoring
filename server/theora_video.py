import asyncio
from aiohttp import web
import json
import base64

import rospy
from theora_image_transport.msg import Packet as TheoraPacket

from .server import ServerBase
from . import utils


class TheoraVideoHandler:

    def __init__(self, srv: ServerBase, config=dict()) -> None:

        # save server object, register our handlers
        self.srv = srv
        self.srv.schedule_task(self.run())
        self.srv.add_route('GET', '/video/get_names', self.get_names_handler, 'video_get_names')
        self.srv.add_route('PUT', '/video/set_stream', self.set_stream_handler, 'video_set_stream')

        # queue for th packets
        self.th_pkt_queue = asyncio.Queue(maxsize=0)
        self.th_hdr_pkt = []

        # subscribers
        self.img_sub = None
        self.img_msg_to_send = {}

    
    @utils.handle_exceptions
    async def get_names_handler(self, request):

        # get topic names from ros master
        topic_name_type_list = await utils.to_thread(rospy.get_published_topics)

        # filter those with theora type
        vs_topics = list()
        for tname, ttype in topic_name_type_list:
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

        if self.img_sub is not None:
            self.img_sub.unregister()
        
        self.th_hdr_pkt.clear()

        self.img_msg_to_send = {}

        if stream_name == '':
            print('disconnected image subscriber')
            return web.Response(text=json.dumps({
                'success': True,
                'message': f'disconnected from image topic"',
                }))
        
        print(f'requested stream {stream_name}, registering subscriber and waiting for headers..')

        self.img_sub = rospy.Subscriber(stream_name, 
            TheoraPacket, self.on_th_pkt_recv, queue_size=20)

        # wait for headers
        iter = 0
        while len(self.th_hdr_pkt) < 3 and iter < 500:
            await asyncio.sleep(0.01)

        if len(self.th_hdr_pkt) < 3:
            return web.Response(text=json.dumps({
                'success': False,
                'message': f'failed to receive headers for "{stream_name}"',
                }))

        print('got headers')

        return web.Response(text=json.dumps({
                'success': True,
                'message': f'subscribed to "{stream_name}"',
                'hdr': self.th_hdr_pkt,
                }))


    
    async def run(self):

        while True:

            # await for a new packet to be received from ros
            th_pkt = await self.th_pkt_queue.get()

            # serialize msg to json
            msg_str = json.dumps(th_pkt)

            # iterate over sockets (one per client)
            await self.srv.ws_send_to_all(msg_str)

    
    def on_th_pkt_recv(self, msg: TheoraPacket):

        th_pkt = dict()
        th_pkt['type'] = 'theora'
        th_pkt['data'] = base64.b64encode(msg.data).decode('ascii')
        th_pkt['b_o_s'] = msg.b_o_s
        th_pkt['e_o_s'] = msg.e_o_s
        th_pkt['granulepos'] = msg.granulepos
        th_pkt['packetno'] = msg.packetno

        if msg.b_o_s == 1:
            self.th_hdr_pkt = [th_pkt]
            print(f'got header {len(self.th_hdr_pkt)}/3')
            return 

        if msg.granulepos == 0:
            self.th_hdr_pkt.append(th_pkt)
            print(f'got header {len(self.th_hdr_pkt)}/3')
            return

        _ = asyncio.run_coroutine_threadsafe(self.th_pkt_queue.put(th_pkt), self.loop)
        
