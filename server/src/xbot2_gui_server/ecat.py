import asyncio
from aiohttp import web
import json
import math
import time

import rospy
from std_srvs.srv import SetBool, Trigger
from ec_srvs.srv import SelectSlave
from std_msgs.msg import Float64

from .server import ServerBase
from . import utils


class EcatHandler:

    def __init__(self, srv: ServerBase, config=dict()) -> None:


        # request ui page
        self.requested_pages = ['Ecat']

        # config
        self.rate = config.get('rate', 10.0)

        # save server object, register our handlers
        self.srv = srv

        self.srv.add_route('POST', '/ecat/start_motor',
                           self.start_motor_handler,
                           'ecat_start_motor_handler')

        self.srv.add_route('POST', '/ecat/stop_motor',
                           self.stop_motor_handler,
                           'ecat_stop_motor_handler')


    @utils.handle_exceptions
    async def stop_motor_handler(self, req):

        name = req.rel_url.query['name']

        srv = rospy.ServiceProxy('/ec_client/stop_motors', SelectSlave)

        res = await utils.to_thread(srv, slave_name=[name])

        if name in res.cmd_status_success_slaves:

            return web.Response(text=json.dumps(
                {
                    'success': True,
                    'message': f'succesfully stopped motor {name}'
                }
                ))

        else:

            return web.Response(text=json.dumps(
                {
                    'success': False,
                    'message': f'failed to stop motor {name}: {res.cmd_status.status}'
                }
                ))


    @utils.handle_exceptions
    async def start_motor_handler(self, req):

        name = req.rel_url.query['name']

        ctrl = req.rel_url.query['ctrl']

        srv = rospy.ServiceProxy(f'/ec_client/start_motors_{ctrl}', SelectSlave)

        res = await utils.to_thread(srv, slave_name=[name])

        if name in res.cmd_status_success_slaves:

            return web.Response(text=json.dumps(
                {
                    'success': True,
                    'message': f'succesfully started motor {name} in {ctrl}'
                }
                ))

        else:

            return web.Response(text=json.dumps(
                {
                    'success': False,
                    'message': f'failed to stopp motor {name} in {ctrl}: {res.cmd_status.status}'
                }
                ))

