import asyncio
from aiohttp import web
import json
import math
import time

from std_srvs.srv import SetBool, Trigger
from std_msgs.msg import Float64
from geometry_msgs.msg import TwistStamped, Twist
#from phase_manager.msg import TimelineArray, Timeline

from .server import ServerBase
from . import utils
from . import ros_utils
ros_handle = ros_utils.ros_handle


class HorizonHandler:

    def __init__(self, srv: ServerBase, config=dict()) -> None:


        # request ui page
        self.requested_pages = ['Horizon']

        # config
        self.rate = config.get('rate', 10.0)

        # save server object, register our handlers
        self.srv = srv
        self.srv.register_ws_coroutine(self.handle_ws_msg)
        self.srv.schedule_task(self.run())

        self.srv.add_route('POST', '/horizon/{gait}/switch',
                           self.walk_switch_handler,
                           'horizon_walk_switch_handler')
        
        # publishers and subscribers using ros_handle
        self.vref_pub = ros_handle.create_publisher(Twist, '/horizon/base_velocity/reference', queue_size=1)
        self.stats_sub = ros_handle.create_subscription(Float64, '/mpc_solution_time', self.sol_time_callback, queue_size=1)
        #self.timeline_sub = ros_handle.create_subscription(TimelineArray, '/phase_manager/timelines', self.timeline_callback, queue_size=1)
        self.solution_time = None
        self.timelines  = None


    @utils.handle_exceptions
    async def walk_switch_handler(self, req):

        gait = req.match_info['gait']

        active = utils.str2bool(req.rel_url.query['active'])

        srv = ros_handle.create_client(SetBool, f'/horizon/{gait}/switch')

        res = await utils.to_thread(srv, data=active)

        return web.Response(text=json.dumps(
            {
                'success': res.success,
                'message': res.message
            }
            ))


    async def run(self):

        while True:

            if self.solution_time is not None:

                msg = {
                    'type': 'horizon_status',
                    'solution_time': self.solution_time,
                    'stamp': time.time(),
                }

                self.solution_time = None

                await self.srv.udp_send_to_all(msg)

            if self.timelines is not None:

                timelines = {}

                for tl in self.timelines.timelines:
                    tl : Timeline = tl 
                    tl_dict = {}
                    tl_dict['phases'] = tl.phases
                    tl_dict['dur'] = tl.durations
                    tl_dict['k0'] = tl.initial_nodes
                    timelines[tl.name] = tl_dict

                msg = {
                    'type': 'horizon_timelines',
                    'timelines': timelines,
                    'stamp': time.time(),
                }

                self.timelines = None

                await self.srv.udp_send_to_all(msg)

            await asyncio.sleep(1./self.rate)


    async def handle_ws_msg(self, msg, proto, ws):
        if msg['type'] == 'horizon_vref':
            rosmsg = Twist()
            vref = msg['vref']
            rosmsg.linear.x = vref[0]
            rosmsg.linear.y = vref[1]
            rosmsg.angular.z = vref[5]
            self.vref_pub.publish(rosmsg)

    
    def sol_time_callback(self, msg):

        self.solution_time = msg.data


    def timeline_callback(self, msg):

        self.timelines = msg


