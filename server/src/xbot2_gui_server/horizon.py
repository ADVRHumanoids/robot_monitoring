import asyncio
from aiohttp import web
import json

import rospy
from std_srvs.srv import SetBool, Trigger
from geometry_msgs.msg import TwistStamped, Twist

from .server import ServerBase
from . import utils


class HorizonHandler:

    def __init__(self, srv: ServerBase, config=dict()) -> None:


        # request ui page
        self.requested_pages = ['Horizon']

        # config
        self.rate = config.get('rate', 1.0)

        # save server object, register our handlers
        self.srv = srv
        self.srv.register_ws_coroutine(self.handle_ws_msg)
        self.srv.schedule_task(self.run())

        self.srv.add_route('POST', '/horizon/walk/switch',
                           self.walk_switch_handler,
                           'horizon_walk_switch_handler')
        
        # subscribers
        self.vref_pub = rospy.Publisher('/horizon/base_velocity/reference', Twist, queue_size=1, tcp_nodelay=True)

        # drill action client
        self.autodrill_client = None


    @utils.handle_exceptions
    async def walk_switch_handler(self, req):

        active = utils.str2bool(req.rel_url.query['active'])

        srv = rospy.ServiceProxy('/horizon/walk/switch', SetBool)

        res = await utils.to_thread(srv, data=active)

        return web.Response(text=json.dumps(
            {
                'success': res.success,
                'message': res.message
            }
            ))


    async def run(self):

        while True:

            await asyncio.sleep(1./self.rate)


    async def handle_ws_msg(self, msg, ws):
        if msg['type'] == 'horizon_vref':
            rosmsg = Twist()
            vref = msg['vref']
            rosmsg.linear.x = vref[0]
            rosmsg.linear.y = vref[1]
            rosmsg.angular.z = vref[5]
            self.vref_pub.publish(rosmsg)



