import asyncio
import rospy
import json
from aiohttp import web
from .server import ServerBase
from . import utils
# DEBUG
from xbot_msgs.msg import Statistics2
from std_srvs.srv import SetBool


class CondaCMD:

    def __init__(self, srv: ServerBase):
        self.rate = 10
        self.srv = srv
        self.srv.schedule_task(self.run())
        self.srv.add_route('POST', '/emergency/kill', self.kill, 'kill')
        self.statusSub = rospy.Subscriber(
            'xbotcore/statistics', Statistics2, self.getStats, queue_size=1)
        self.currentState = ''

    @utils.handle_exceptions
    async def kill(self, request):
        killer = rospy.ServiceProxy('xbotcore/cartesio_imp/switch', SetBool)
        res = await utils.to_thread(killer, False)
        return web.Response(text=json.dumps(
            {
                'success': res.success,
                'message': "Robot Killed",
            }))

    def getStats(self, msg: Statistics2):
        for p in msg.task_stats:
            if (p.name == 'cartesio_imp'):
                self.currentState = p.state

    async def run(self):
        while True:
            msg = {
                'type': 'robot_state',
                'state': ''
            }
            if self.currentState == 'Running':
                print('Robot Alive')
                msg['state'] = 'Alive'
            await self.srv.udp_send_to_all(msg)
            await asyncio.sleep(1./self.rate)
