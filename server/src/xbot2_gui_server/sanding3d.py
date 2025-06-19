import rospy
from geometry_msgs.msg import TransformStamped, Transform, Quaternion, Pose
import actionlib
import json

try:
    from concert_sanding.msg import ScanAction
    # from concert_bts.msg import DetectWallAction
    from concert_sanding.msg import ScanGoal
    # from concert_bts.msg import DetectWallGoal
    from concert_sanding.msg import ScanActionFeedback
    # from concert_bts.msg import DetectWallActionFeedback
    from concert_sanding.msg import Wall
    from concert_sanding.msg import ScanResult
    # from concert_bts.msg import DetectWallResult
    from concert_sanding.srv import GetWall

    from concert_sanding.msg import ApproachWallAction
    from concert_sanding.msg import ApproachWallFeedback
    from concert_sanding.msg import ApproachWallGoal

except ModuleNotFoundError:
    pass

import asyncio
from aiohttp import web
import numpy as np
from scipy.spatial.transform import Rotation as R
from std_srvs.srv import SetBool, Trigger
from std_msgs.msg import String, Int16

from .server import ServerBase
from . import utils


class Sanding3DHandler:
    def __init__(self, srv: ServerBase, config=dict()) -> None:
        self.wall_poses = {}
        self.requested_pages = ['Sanding3D']
        self.rate = config.get('rate', 200.0)

        self.sanding_tool_activated_service = config['sanding_tool_activated_service']
        
        self.srv = srv
        self.srv.schedule_task(self.run())
        self.srv.add_route('POST', '/sanding/start_scanning',
                           self.start_scanning,
                           'sanding_start_scanning')
        self.srv.add_route('POST', '/sanding/upload_params',
                           self.upload_params,
                           'upload_params')

        self.srv.add_route('POST', '/sanding/approach_wall',
                           self.approach_wall,
                           'approach_wall')
        
        self.srv.add_route('POST', '/concert/sanding/tool_started_ack',
                           self.sanding_tool_started_ack_handler,
                           'concert_sanding_tool_started_ack_handler')
        
        self.srv.add_route('POST', '/concert/sanding/kill_all',
                            self.kill_all_handler,
                            'concert_sanding_kill_all_handler')

        self.scanning_progress = None
        self.wall_poses = dict()

        # ToDo: load from config
        self.map_sub = rospy.Subscriber(
            "/concert_mapping/pose", TransformStamped, self.on_map_recv)
        # self.map_pose = None
        self.map_translation = None
        self.map_rotation = None

        # sanding subscribers
        self.sanding_status = None 
        self.sanding_progress = None
        self.sanding_status_sub = rospy.Subscriber(config['sanding_status_topic'], String, self.sanding_status_recv)
        self.sanding_progress_sub = rospy.Subscriber(config['sanding_progress_topic'], Int16, self.sanding_progress_recv)

    async def run(self):

        while True:
            
            if self.sanding_progress is not None or self.sanding_status is not None:

                msg = {
                    'type': 'concert_sanding_progress',
                    'progress': self.sanding_progress if self.sanding_progress is not None else -1,
                    'status': self.sanding_status if self.sanding_status is not None else '--',
                }

                self.sanding_progress = None
                self.sanding_status = None
                await self.srv.udp_send_to_all(msg)

            if self.map_translation is not None and self.map_rotation is not None:
                msg = {
                    'type': 'map',
                    'transform': {
                        'position': {
                            'x': self.map_translation[0],
                            'y': self.map_translation[1],
                            'z': self.map_translation[2]
                        },
                        'orientation': {
                            'x': self.map_rotation.as_quat()[0],
                            'y': self.map_rotation.as_quat()[1],
                            'z': self.map_rotation.as_quat()[2],
                            'w': self.map_rotation.as_quat()[3]
                        }
                    }
                }
                # self.map_pose = None
                self.map_translation = None
                self.map_rotation = None
                await self.srv.udp_send_to_all(msg)
            await asyncio.sleep(1/self.rate)

    def on_map_recv(self, msg: TransformStamped):
        # self.map_pose = Transform() # msg.transform

        if np.linalg.norm([msg.transform.rotation.x, msg.transform.rotation.y,
                           msg.transform.rotation.z, msg.transform.rotation.w]) < 0.01:
            print('[wall_detection] Map pose is not set yet')
            # setting zero for translation and rotation
            # Better to None
            self.map_translation = np.array([0.0, 0.0, 0.0])
            self.map_rotation = R.from_quat([0.0, 0.0, 0.0, 1.0])
            return
        qml_translation, qml_rotation = self.toQMLFrame(
            msg.transform.translation, msg.transform.rotation)
        self.map_translation = qml_translation
        self.map_rotation = qml_rotation

    @utils.handle_exceptions
    async def start_scanning(self, req: web.Request):
        # Clear wall dict
        self.wall_poses = dict()
        scanClient = actionlib.SimpleActionClient(
            '/concert_sanding/scan', ScanAction)
        print('[wall_detection] waiting for server...')
        ok = await utils.to_thread(scanClient.wait_for_server, timeout=rospy.Duration(3.0))
        # print(ok)
        goal = ScanGoal()
        params = await req.json()
        print(F'Angle: {params}')
        goal.angle = params
        fb_last: ScanActionFeedback = None

        def on_feedback(fb: ScanActionFeedback):
            nonlocal fb_last
            fb_last = fb

        scanClient.send_goal(goal, feedback_cb=on_feedback)
        scanning = True

        while scanning:
            if scanClient.get_state() == 2:
                return web.Response(text=json.dumps(
                    {
                        'success': False,
                        'message': 'Scanning cancelled',
                    }))

            if fb_last is not None:
                await self.srv.udp_send_to_all({
                    'type': 'scanning_progress',
                    'progress': fb_last.progress
                })
                if fb_last.progress >= 100:
                    scanning = False
            await asyncio.sleep(0.1)

        
        scanClient.wait_for_result()

        result = scanClient.get_result()
        # print(f'Mission success: {result.success}')
        # return web.Response(text=json.dumps(
        #     {
        #         'success': result.success,
        #         'message': 'Scanning completed',
        #     }))
    
        if result.success:
            wallClient = rospy.ServiceProxy('/concert_sanding/get_wall', GetWall)
            ok = await utils.to_thread(wallClient.wait_for_service, timeout=rospy.Duration(3.0))
            wallResult = await utils.to_thread(wallClient)
            walls = []
            for w in wallResult.wall:
                self.wall_poses[w.id] = w.pose
                '''
                current_orientation = np.array(
                    [w.pose.orientation.x, w.pose.orientation.y, w.pose.orientation.z, w.pose.orientation.w])
                print(
                    f'Wall {w.id} Initial position: {w.pose.position.x}, {w.pose.position.y}, {w.pose.position.z}')
                euler_angles = np.array([-90, 0, 0])
                rotation_quaternion = R.from_euler(
                    'xyz', euler_angles, degrees=True).as_quat()
                new_orientation = R.from_quat(
                    current_orientation) * R.from_quat(rotation_quaternion)
                # new_orientation_quat = new_orientation.as_quat()
                gazebo_position = np.array(
                    [w.pose.position.x, w.pose.position.y, w.pose.position.z])
                gazebo_orientation = np.array(
                    [w.pose.orientation.x, w.pose.orientation.y, w.pose.orientation.z, w.pose.orientation.w])
                gazebo_rotation = R.from_quat(gazebo_orientation)
                # qml_rotation = R.from_euler("xyz", [0, 90, 90], degrees=True)
                combined_rotation = R.from_euler(
                    'z', 90, degrees=True) * R.from_euler('y', 90, degrees=True)

                qml_position = combined_rotation.apply(gazebo_position)
                qml_orietation = combined_rotation * gazebo_rotation
                '''

                qml_translation, qml_orietation = self.toQMLFrame(
                    w.pose.position, w.pose.orientation)
                walls.append({
                    'id': w.id,
                    'index': w.index,
                    'pose': {
                        'position': {
                            'x': qml_translation[0],  # w.pose.position.x,
                            'y': qml_translation[1],  # w.pose.position.y,
                            'z': qml_translation[2]  # w.pose.position.z
                        },
                        'orientation': {
                            'x': qml_orietation.as_quat()[0],
                            'y': qml_orietation.as_quat()[1],
                            'z': qml_orietation.as_quat()[2],
                            'w': qml_orietation.as_quat()[3]
                        }
                    },
                    'length': w.length,
                    'type': w.type,
                })
            msg = {
                'type': 'wall_list',
                'walls': walls
            }
            await self.srv.udp_send_to_all(msg)

        return web.Response(text=json.dumps(
            {
                'success': True,
                'message': 'Scanning started',
            }))

    @utils.handle_exceptions
    async def upload_params(self, req: web.Request):
        params = await req.json()
        print(f'Parameters: {params}')
        cornerY = params['x']
        sanderRadius = 0.12
        if params['type'] == 'left':
            cornerY -= sanderRadius
        

        rospy.set_param('/sanding/force', params['force'])
        rospy.set_param('/sanding/length', params['width'])
        rospy.set_param('/sanding/height', params['height'])
        rospy.set_param('/sanding/corner_y', cornerY)
        rospy.set_param('/sanding/corner_z', params['y'])
        rospy.set_param('/sanding/index', params['index'])
        rospy.set_param('/sanding/type', "center")
        rospy.set_param('/sanding/robot_type', "6dof-40")

        await asyncio.sleep(1.0)

        return web.Response(text=json.dumps(
            {
                'success': True,
                'message': 'Ros Params Uploaded',
            }))

    @utils.handle_exceptions
    async def approach_wall(self, req: web.Request):
        
        client = actionlib.SimpleActionClient(
            '/concert_sanding/approach_wall', ApproachWallAction)
        ok = await utils.to_thread(client.wait_for_server, timeout=rospy.Duration(3.0))
        goal = ApproachWallGoal()
        id = await req.json()
        target = self.wall_poses[id]

        goal.target = target
        print(f'Reaching wall with id: {id}')
        
        fb_last: ApproachWallFeedback = None

        def on_feedback(fb: ApproachWallFeedback):
            nonlocal fb_last
            
            fb_last = fb
            
        
        client.send_goal(goal, feedback_cb=on_feedback)
        
        # non blocking wait for result
        # await utils.to_thread(client.wait_for_result, timeout=rospy.Duration(30.0))
        completed = False
        while not completed:
            print(f'client state: {client.get_state()}')
            if client.get_state() == 2:
                return web.Response(text=json.dumps(
                    {
                        'success': False,
                        'message': 'Approach cancelled',
                    }))
            if fb_last is not None:
                await self.srv.udp_send_to_all({
                    'type': 'approach_state',
                    'status': fb_last.status
                })
                if fb_last.status == "completed":
                    completed = True
            
            await asyncio.sleep(0.1)
            
        client.wait_for_result()
        result = client.get_result()
        if result.success:
            print(f'Approached wall with id: {id}')
            return web.Response(text=json.dumps(
                {
                    'success': True,
                    'message': "Wall Approached",
                }))


    def toQMLFrame(self, inputPosition, inputRotation):

        gazebo_position = np.array(
            [inputPosition.x, inputPosition.y, inputPosition.z])
        gazebo_rotation = np.array(
            [inputRotation.x, inputRotation.y, inputRotation.z, inputRotation.w])
        gazebo_rotation = R.from_quat(gazebo_rotation)

        combined_rotation = R.from_euler(
            'z', 90, degrees=True) * R.from_euler('y', 90, degrees=True)

        qml_position = combined_rotation.apply(gazebo_position)
        qml_rotation = combined_rotation * gazebo_rotation

        return qml_position, qml_rotation
    
    @utils.handle_exceptions
    async def sanding_tool_started_ack_handler(self, req: web.Request):

        print('drill tool has been started!!')

        srv = rospy.ServiceProxy(self.sanding_tool_activated_service, 
                                 Trigger)
        
        res = await utils.to_thread(srv)

        return web.Response(text=json.dumps(
            {
                'success': res.success,
                'message': res.message,
            }))
    
    def sanding_status_recv(self, msg: String):
        self.sanding_status = msg.data
    
    def sanding_progress_recv(self, msg: Int16):
        self.sanding_progress = msg.data
    
    @utils.handle_exceptions
    async def kill_all_handler(self, req: web.Request):

        print('Cancelling all goals')
        scanningClient = actionlib.SimpleActionClient(
            '/concert_sanding/scan', ScanAction)
        wallApproachClient = actionlib.SimpleActionClient(
            '/concert_sanding/approach_wall', ApproachWallAction)
        
        scanningClient.cancel_all_goals()
        wallApproachClient.cancel_all_goals()
        print('All goals cancelled')
        return web.Response(text=json.dumps(
            {
                'success': True,
                'message': 'All goals cancelled',
            }))
    