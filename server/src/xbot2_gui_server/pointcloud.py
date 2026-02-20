import asyncio
from aiohttp import web
import json

import rospy
from urdf_parser_py import urdf as urdf_parser
import tf
from scipy.spatial.transform import Rotation as R

from .server import ServerBase
from . import utils

from sensor_msgs import point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, Range

from threading import Lock
from functools import partial

from .proto import jointstate_pb2, generic_pb2

class PointCloudHandler:

    def __init__(self, srv: ServerBase, config=dict()) -> None:

        # config
        self.rate = config.get('rate', 10.0)

        # save server object, register our handlers
        self.srv = srv

        self.srv.add_route('GET', '/visual/get_pointcloud',
                           self.visual_get_pc,
                           'visual_get_pc')     

        self.srv.add_route('GET', '/visual/get_sonar',
                           self.visual_get_sonar,
                           'visual_get_sonar')   
        
        self.srv.schedule_task(self.run())

        self.srv.register_ws_coroutine(self.handle_ws)

        # point clouds (todo: detect)
        self.pc_lock = Lock()
        pc_topics = ['/velodyne_points']
        self.pc_subs = [rospy.Subscriber(t, PointCloud2, self.on_pc_recv, t, queue_size=1) for t in pc_topics]      
        self.pc_map = {t: [] for t in pc_topics}
        self.pc_frame_map = {t: None for t in pc_topics}
        self.pc_client_sock = set()
        
        # sonar topics (todo: detect)
        sonar_topics = []
        self.sonar_subs = [rospy.Subscriber(t, Range, self.on_sonar_recv, t, queue_size=1) for t in sonar_topics]
        self.sonar_map = {t: -1.0 for t in sonar_topics}
        self.sonar_frame_map = {t: None for t in sonar_topics}

        
    async def handle_ws(self, msg, proto, ws):
        if msg['type'] == 'pc_registration':
            if ws not in self.pc_client_sock:
                self.pc_client_sock.add(ws)
                print(f'registered client id {ws}: total is {len(self.pc_client_sock)}')
        elif msg['type'] == 'pc_unregistration':
            self.pc_client_sock.discard(ws)
    

    @utils.handle_exceptions
    async def visual_get_pc(self, req: web.Request):
        
        tfl = tf.TransformListener()

        res = {}
        
        for pcname, pcframe in self.pc_frame_map.items():
            await utils.to_thread(tfl.waitForTransform, source_frame=pcframe, target_frame='base_link', time=rospy.Time(0), timeout=rospy.Duration(2.0))
            pos, rot = tfl.lookupTransform(source_frame=pcframe, target_frame='base_link', time=rospy.Time(0))
            print(f'{pcname} {pcframe} {pos} {rot}')
            res[pcname] = dict(pos=pos, rot=rot)

        del tfl

        return web.json_response(res)

    
    @utils.handle_exceptions
    async def visual_get_sonar(self, req: web.Request):
        
        tfl = tf.TransformListener()

        res = {}
        
        for sname, sframe in self.sonar_frame_map.items():
            await utils.to_thread(tfl.waitForTransform, source_frame=sframe, target_frame='base_link', time=rospy.Time(0), timeout=rospy.Duration(2.0))
            pos, rot = tfl.lookupTransform(source_frame=sframe, target_frame='base_link', time=rospy.Time(0))
            print(f'{sname} {sframe} {pos} {rot}')
            res[sname] = dict(pos=pos, rot=rot)

        del tfl

        return web.json_response(res)


    async def run_loop(self):

        # broadcast sonar if any
        await self.srv.udp_send_to_all({
                'type': 'sonar',
                'range': self.sonar_map
            })
                    
        # filter pc recipients to only contain alive clients
        self.pc_client_sock = {cli_sock for cli_sock in self.pc_client_sock if cli_sock in self.srv.udp_clients}
        
        # broadcast point cloud to subscribed clients
        with self.pc_lock:
            for k, v in self.pc_map.items():
                for i, m in enumerate(v):
                    await self.srv.udp_send_to_all(m, clients=self.pc_client_sock)
                self.pc_map[k] = []
                

    async def run(self):

        async def print_err(msg):
            print(msg)

        wrapped = utils.sync_loop(self.run_loop, dt=1./self.rate, on_exception=print_err)
        await wrapped()

    
    def on_sonar_recv(self, msg: Range, sname):

        self.sonar_frame_map[sname] = msg.header.frame_id
        self.sonar_map[sname] = msg.range


    def on_pc_recv(self, msg: PointCloud2, pcname):
        
        self.pc_frame_map[pcname] = msg.header.frame_id

        if len(self.pc_client_sock) == 0:
            return

        # get xyz from point cloud
        points = pc2.read_points(msg, field_names=('x', 'y', 'z'))
        
        # add all points, break msg when big enough
        max_xyz_size = 1000 // 4  # 4 bytes per point, 1000 bytes max 
        msg = None
        msgs = []
        for pt in points:
            
            # init msg if needed
            if msg is None:
                msg = generic_pb2.Message()
                msg.point_cloud.name = pcname
                msg.point_cloud.iblk = len(msgs)
            
            # add xyz from current point
            msg.point_cloud.xyz.extend(pt)
            
            # commit msg if large enough
            if len(msg.point_cloud.xyz) > max_xyz_size:
                msgs.append(msg)
                msg = None

        # fill nblk
        nblk = len(msgs)
        for m in msgs:
            m.point_cloud.nblk = nblk
            
        print(f'Recv PC, split in nblk={nblk}')
        
        # save msgs to map
        with self.pc_lock:
            self.pc_map[pcname] = msgs
    
    



