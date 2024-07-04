import asyncio
from aiohttp import web
import json

import rospy
import actionlib
import tf

try:
    from concert_vision.msg import BlobArray, Blob
    from concert_autonomous_drilling.msg import AutoDrillAction
    from concert_autonomous_drilling.msg import AutoDrillGoal
    from concert_autonomous_drilling.msg import AutoDrillFeedback
    from concert_autonomous_drilling.msg import AutoDrillResult
    concert_drilling_found = True
except ModuleNotFoundError:
    concert_drilling_found = False
    class BlobArray:
        pass


from std_srvs.srv import SetBool, Trigger
from std_msgs.msg import Float64, String, Int16
from geometry_msgs.msg import TwistStamped, Twist

from .server import ServerBase
from . import utils


class ConcertHandler:

    def __init__(self, srv: ServerBase, config=dict()) -> None:

        # request ui page
        self.requested_pages = ['Builder', 'Linfa', 'Drill Task', 'Sanding', 'Transportation']

        # config
        self.rate = config.get('rate', 10.0)
        self.sanding_tool_activated_service = config['sanding_tool_activated_service']

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
        
        self.srv.add_route('POST', '/concert/sanding/start',
                           self.sanding_start_handler,
                           'concert_sanding_start_handler')
        
        self.srv.add_route('GET', '/concert/sanding/configure',
                           self.sanding_configure_handler,
                           'concert_sanding_configure')
        
        self.srv.add_route('POST', '/concert/sanding/tool_started_ack',
                           self.sanding_tool_started_ack_handler,
                           'concert_sanding_tool_started_ack_handler')
        
        # drilling subscribers
        if concert_drilling_found:
            self.markers_sub = rospy.Subscriber(config['blob_array_topic'], BlobArray, self.blob_array_recv, queue_size=1)
        
        self.last_recv_marker : BlobArray = None

        # sanding subscribers
        self.sanding_status = None 
        self.sanding_progress = None
        self.sanding_status_sub = rospy.Subscriber(config['sanding_status_topic'], String, self.sanding_status_recv)
        self.sanding_progress_sub = rospy.Subscriber(config['sanding_progress_topic'], Int16, self.sanding_progress_recv)

        # drill action client
        self.autodrill_client = None

    
    def blob_array_recv(self, msg: BlobArray):
        self.last_recv_marker = msg


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
        task_name = req.rel_url.query['task_name']

        from cartesian_interface.srv import SetControlMode

        srv = rospy.ServiceProxy(f'/cartesian/{task_name}/set_control_mode', SetControlMode)

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


    def sanding_status_recv(self, msg: String):
        self.sanding_status = msg.data


    def sanding_progress_recv(self, msg: Int16):
        self.sanding_progress = msg.data


    @utils.handle_exceptions
    async def sanding_configure_handler(self, req: web.Request):

        return web.Response(text=json.dumps(
            {
                'success': True,
                'message': 'got sanding configuration',
                'breakpoints': {
                    '6dof-40-40': [0., 1., 2., 2.4],
                    '6dof-60-40-40': [1., 2., 3.],
                }
            })) 


    @utils.handle_exceptions
    async def sanding_start_handler(self, req: web.Request):

        body = await req.json()

        print(body)

        rospy.set_param('/sanding/force', body['force'])
        rospy.set_param('/sanding/length', body['xmax'] - body['xmin'])
        rospy.set_param('/sanding/height', body['ymax'] - body['ymin'])
        rospy.set_param('/sanding/corner_y', -body['xmin'])
        rospy.set_param('/sanding/corner_z', body['ymax'])
        rospy.set_param('/sanding/type', body['type'])
        rospy.set_param('/sanding/index', body['height_level'])

        await asyncio.sleep(1.0)

        return web.Response(text=json.dumps(
            {
                'success': True,
                'message': 'sanding started',
            }))


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
