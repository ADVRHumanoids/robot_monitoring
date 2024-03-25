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

        # request ui page
        self.requested_pages = ['Builder', 'Linfa', 'Drill Task']

        # config
        self.rate = config.get('rate', 10.0)

        # save server object, register our handlers
        self.srv = srv
        self.srv.register_ws_coroutine(self.handle_ws_msg)
        self.srv.schedule_task(self.run())

        self.srv.add_route('POST', '/concert/do_drill',
                           self.do_drill_handler,
                           'concert_do_drill')
        
        self.srv.add_route('POST', '/concert/abort_drill',
                           self.abort_drill_handler,
                           'concert_abort_drill')
        
        self.srv.add_route('POST', '/concert/enable_arm',
                           self.enable_arm_handler,
                           'concert_enable_arm')
        
        self.srv.add_route('POST', '/concert/gcomp_switch',
                           self.gcomp_switch,
                           'concert_gcomp_switch')
        
        # subscribers
        self.markers_queue = asyncio.Queue()
        self.markers_sub = rospy.Subscriber(config['blob_array_topic'], BlobArray, self.blob_array_recv, queue_size=1)
        self.last_recv_marker : BlobArray = None

        # drill action client
        self.autodrill_client = None

    
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

                await self.srv.udp_send_to_all(msg)

            await asyncio.sleep(1./self.rate)

    @utils.handle_exceptions
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
    async def gcomp_switch(self, req):

        active = utils.str2bool(req.rel_url.query['active'])

        gcomp_switch = rospy.ServiceProxy('/cartesio/gcomp_switch', SetBool)

        print('[gcomp_switch] waiting for server...')
        ok = await utils.to_thread(gcomp_switch.wait_for_service, timeout=rospy.Duration(3.0))
       
        res = await utils.to_thread(gcomp_switch, active)

        return web.Response(text=json.dumps(
            {
                'success': res.success,
                'message': res.message,
            }))


    @utils.handle_exceptions
    async def do_drill_handler(self, req):

        blob_id = req.rel_url.query['blob_id']
        blob_depth = float(req.rel_url.query['blob_depth'])
        drill_velocity = float(req.rel_url.query['drill_velocity'])

        client = actionlib.SimpleActionClient('/auto_drill_node/auto_drill', AutoDrillAction)
        self.autodrill_client = client
        
        print('[auto_drill] waiting for server...')
        ok = await utils.to_thread(client.wait_for_server, timeout=rospy.Duration(3.0))
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
        goal.approach_velocity = 0.03
        goal.pre_approach_distance = 0.30
        goal.force_threshold = 40.0
        goal.blob_depth = blob_depth
        goal.drill_velocity = drill_velocity

        print(goal)
        
        # fb callback
        fb_last: AutoDrillFeedback = None

        def on_feedback(fb: AutoDrillFeedback):
            nonlocal fb_last
            fb_last = fb
        
        client.send_goal(goal, feedback_cb=on_feedback)

        while True:
            
            if fb_last is not None:
                await self.srv.udp_send_to_all({
                    'type': 'concert_drill_progress',
                    'status': fb_last.current_phase,
                    'error_x': fb_last.servo_fb.error_x,
                    'error_y': fb_last.servo_fb.error_y,
                    'force': fb_last.servo_fb.wrench.wrench.force.z,
                    })
                            
            if client.get_state() > 2:
                break

            await asyncio.sleep(0.1)

        self.autodrill_client = None
                
        client.wait_for_result()

        res = client.get_result()

        return web.Response(text=json.dumps(
            {
                'success': res.success,
                'message': res.message,
            }))
    

    @utils.handle_exceptions
    async def abort_drill_handler(self, req):

        if self.autodrill_client is None:
            return web.Response(text=json.dumps(
            {
                'success': False,
                'message': 'action is not active',
            }))
        
        self.autodrill_client.cancel_all_goals()

        return web.Response(text=json.dumps(
            {
                'success': True,
                'message': 'cancel done',
            }))


    async def handle_ws_msg(self, msg, proto, ws):
        if msg['type'] == 'concert_drill_vref':
            await self.handle_velocity_command(msg)


    async def handle_velocity_command(self, msg):
        print(msg)


