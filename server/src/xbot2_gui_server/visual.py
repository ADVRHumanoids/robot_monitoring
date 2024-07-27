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

class RoundingFloat(float):
    __repr__ = staticmethod(lambda x: format(x, '.2f'))

json.encoder.c_make_encoder = None
json.encoder.float = RoundingFloat

class VisualHandler:

    def __init__(self, srv: ServerBase, config=dict()) -> None:

        # config
        self.rate = config.get('rate', 10.0)

        # save server object, register our handlers
        self.srv = srv

        self.srv.add_route('GET', '/visual/get_mesh/{uri}',
                           self.visual_get_mesh_handler,
                           'visual_get_mesh_handler')

        self.srv.add_route('GET', '/visual/get_mesh_entities',
                           self.visual_get_mesh_entities,
                           'visual_get_mesh_entities')

        self.srv.add_route('GET', '/visual/get_mesh_tfs',
                           self.visual_get_mesh_tfs,
                           'visual_get_mesh_tfs')   

        self.srv.add_route('GET', '/visual/get_pointcloud',
                           self.visual_get_pc,
                           'visual_get_pc')     

        self.srv.add_route('GET', '/visual/get_sonar',
                           self.visual_get_sonar,
                           'visual_get_sonar')   
        
        self.srv.schedule_task(self.run())

        # # point clouds
        # self.pc_lock = Lock()
        # pc_topics = ['/VLP16_lidar_back/velodyne_points', '/VLP16_lidar_front/velodyne_points']
        # self.pc_subs = [rospy.Subscriber(t, PointCloud2, self.on_pc_recv, t, queue_size=1) for t in pc_topics]      
        # self.pc_map = {t: None for t in pc_topics}
        # self.pc_frame_map = {t: None for t in pc_topics}
        
        # # sonar topics
        # sonar_topics = [
        #     '/bosch_uss5/ultrasound_fl_lat',      
        #     '/bosch_uss5/ultrasound_fl_sag',      
        #     '/bosch_uss5/ultrasound_fr_lat',      
        #     '/bosch_uss5/ultrasound_fr_sag',      
        #     '/bosch_uss5/ultrasound_rl_lat',      
        #     '/bosch_uss5/ultrasound_rl_sag',      
        #     '/bosch_uss5/ultrasound_rr_lat',      
        #     '/bosch_uss5/ultrasound_rr_sag',  
        #     ]
        
        # self.sonar_subs = [rospy.Subscriber(t, Range, self.on_sonar_recv, t, queue_size=1) for t in sonar_topics]
        # self.sonar_map = {t: None for t in sonar_topics}
        # self.sonar_frame_map = {t: None for t in sonar_topics}

        

    
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


    async def run(self):

        while True:
            
            await self.srv.udp_send_to_all({
                'type': 'sonar',
                'range': self.sonar_map
            })
            
            with self.pc_lock:
                for k, v in self.pc_map.items():
                    if v is None: continue
                    try:
                        await self.srv.ws_send_to_all(v) 
                    except BaseException as e:
                        print(type(e), e)
                        continue

            await asyncio.sleep(0.1)

    
    def on_sonar_recv(self, msg: Range, sname):

        self.sonar_frame_map[sname] = msg.header.frame_id
        self.sonar_map[sname] = msg.range


    def on_pc_recv(self, msg: PointCloud2, pcname):

        self.pc_frame_map[pcname] = msg.header.frame_id

        points = list(pc2.read_points(msg, field_names=('x', 'y', 'z')))[::1]

        msg = json.dumps(
                {
                    'points': points,
                    'name': pcname,
                    'type': 'pointcloud'
                })

        with self.pc_lock:
        
            self.pc_map[pcname] = msg
    
    
    @utils.handle_exceptions
    async def visual_get_mesh_handler(self, request):
        uri = request.match_info['uri']
        path = utils.resolve_ros_uri(uri)
        print('URI/PATH: ', uri, path)
        return web.FileResponse(path)

    
    @utils.handle_exceptions
    async def visual_get_mesh_entities(self, request):
        
        # parse urdf
        urdf = rospy.get_param('xbotcore/robot_description')
        urdf = urdf.replace('<texture/>', '')
        model = urdf_parser.Robot.from_xml_string(urdf)

        # get list of visuals
        visuals = dict()
        for lname, l in model.link_map.items():
            
            if l.collision is None:
                continue

            if l.collision.origin is None:
                origin = {
                    'origin_xyz': [0, 0, 0],
                    'origin_rot': [0, 0, 0, 1]
                }
            else:
                origin = {
                    'origin_xyz': l.collision.origin.xyz,
                    'origin_rot': R.from_euler('XYZ', l.collision.origin.rpy).as_quat().tolist()
                }
            
            for c in l.collisions:
                if isinstance(c.geometry, urdf_parser.Mesh):
                    visuals[lname] = {
                        **origin,
                        'type': 'MESH',
                        'filename': c.geometry.filename,
                        'scale': c.geometry.scale,
                    }
                elif isinstance(c.geometry, urdf_parser.Cylinder):
                    visuals[lname] = {
                        **origin,
                        'type': 'CYLINDER',
                        'filename': '#CYLINDER',
                        'radius': c.geometry.radius*1000,
                        'length': c.geometry.length*1000,
                        'scale': [0.001, 0.001, 0.001]
                    }
        
        return web.json_response(visuals)


    @utils.handle_exceptions
    async def visual_get_mesh_tfs(self, request: web.Request):

        # tf listener
        tfl = tf.TransformListener()

        # parse urdf
        urdf = rospy.get_param('xbotcore/robot_description')
        urdf = urdf.replace('<texture/>', '')
        model = urdf_parser.Robot.from_xml_string(urdf)
        root = 'base_link'

        # get list of visuals
        transforms = dict()
        for lname, l in model.link_map.items():
            
            for c in l.collisions:
                if isinstance(c.geometry, urdf_parser.Mesh):
                    await utils.to_thread(tfl.waitForTransform, lname, root, rospy.Time(0), timeout=rospy.Duration(1.0))
                    trans, rot = tfl.lookupTransform(root, lname, rospy.Time(0))
                    transforms[lname] = [trans, rot]
        
        return web.json_response(transforms)



