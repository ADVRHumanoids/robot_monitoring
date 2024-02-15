import asyncio
from aiohttp import web
import json

import rospy
import actionlib
import tf

from concert_vision.msg import BlobArray, Blob

from concert_autonomous_drilling.msg import AutoDrillAction
from concert_autonomous_drilling.msg import AutoDrillGoal
from concert_autonomous_drilling.msg import AutoDrillFeedback
from concert_autonomous_drilling.msg import AutoDrillResult

from std_srvs.srv import SetBool, Trigger
from geometry_msgs.msg import TwistStamped, Twist

from .server import ServerBase
from . import utils


class ConcertHandler:

    def __init__(self, srv: ServerBase, config=dict()) -> None:

        # config
        self.rate = config.get('rate', 10.0)

        # save server object, register our handlers
        self.srv = srv
        self.srv.register_ws_coroutine(self.handle_ws_msg)
        self.srv.schedule_task(self.run())

        self.srv.add_route('POST', '/concert/do_drill',
                           self.do_drill_handler,
                           'concert_do_drill')
        
        self.srv.add_route('POST', '/concert/enable_arm',
                           self.enable_arm_handler,
                           'concert_enable_arm')
        
        # subscribers
        self.markers_queue = asyncio.Queue()
        self.markers_sub = rospy.Subscriber(config['blob_array_topic'], BlobArray, self.blob_array_recv, queue_size=1)
        self.last_recv_marker : BlobArray = None

    
    def blob_array_recv(self, msg: BlobArray):
        self.last_recv_marker = msg


    async def run(self):

        while True:
            
            if self.last_recv_marker is not None:

                msg = {
                    'type': 'concert_blob_array',
                    'blobs': []
                }

                for blob in self.last_recv_marker.blobs:

                    msg['blobs'].append({
                            'id': blob.blob_id
                        })
                    
                self.last_recv_marker = None

                await self.srv.ws_send_to_all(msg)

            await asyncio.sleep(1./self.rate)


    async def enable_arm_handler(self, req):

        active = utils.str2bool(req.rel_url.query['active'])

        from cartesian_interface.srv import SetControlMode

        srv = rospy.ServiceProxy('/cartesian/ee_E/set_control_mode', SetControlMode)

        if active: 
            res = await utils.to_thread(srv, ctrl_mode='velocity')
        else:
            res = await utils.to_thread(srv, ctrl_mode='position')

        return web.Response(text=json.dumps(
            {
                'success': res.success,
                'message': res.message
            }
            ))

    @utils.handle_exceptions
    async def do_drill_handler(self, req):

        blob_id = req.rel_url.query['blob_id']

        client = actionlib.SimpleActionClient('/auto_drill_node/auto_drill', AutoDrillAction)
        
        print('[auto_drill] waiting for server...')
        ok = await utils.to_thread(client.wait_for_server, timeout=rospy.Duration(1.0))
        print(ok)

        if not ok:
            return web.Response(text=json.dumps(
            {
                'success': False,
                'message': 'server unavailable'
            }))
        
        # goal
        goal = AutoDrillGoal()
        goal.blob_id = blob_id
        goal.approach_velocity = 0.1
        goal.pre_approach_distance = 0.40
        goal.force_threshold = 20.0
        
        # fb callback
        fb_last: AutoDrillFeedback = None

        def on_feedback(fb: AutoDrillFeedback):
            nonlocal fb_last
            fb_last = fb
        
        client.send_goal(goal, feedback_cb=on_feedback)

        while True:
            
            if fb_last is not None:
                await self.srv.ws_send_to_all({
                    'type': 'concert_drill_progress',
                    'status': fb_last.current_phase,
                    'error_x': fb_last.servo_fb.error_x,
                    'error_y': fb_last.servo_fb.error_y,
                    'force': fb_last.servo_fb.wrench.wrench.force.z,
                    })
                            
            if client.get_state() > 2:
                break

            await asyncio.sleep(0.1)
                
        client.wait_for_result()

        res = client.get_result()

        return web.Response(text=json.dumps(
            {
                'success': res.success,
                'message': res.message,
            }))


    async def handle_ws_msg(self, msg, ws):
        if msg['type'] == 'concert_drill_vref':
            await self.handle_velocity_command(msg)


    async def handle_velocity_command(self, msg):
        print(msg)


