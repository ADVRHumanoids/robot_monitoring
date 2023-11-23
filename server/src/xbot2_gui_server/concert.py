import asyncio
from aiohttp import web
import json

import rospy
from std_srvs.srv import SetBool, Trigger
from geometry_msgs.msg import TwistStamped, Twist

from .server import ServerBase
from . import utils


class ConcertHandler:

    def __init__(self, srv: ServerBase, config=dict()) -> None:

        # config
        # self.rate = config.get('rate', 10.0)

        # save server object, register our handlers
        self.srv = srv
        self.srv.register_ws_coroutine(self.handle_ws_msg)
        self.srv.schedule_task(self.run())

        self.srv.add_route('POST', '/concert/do_drill',
                           self.do_drill_handler,
                           'concert_do_drill')


    async def run(self):
        pass


    async def do_drill_handler(self, req):
        print('DRILL!!!!!')
        return web.Response(text=json.dumps(
            {
                'success': True,
                'message': 'ok',
            }
        ))


    async def handle_ws_msg(self, msg, ws):
        if msg['type'] == 'concert_drill_vref':
            await self.handle_velocity_command(msg)


    async def handle_velocity_command(self, msg):
        print(msg)


