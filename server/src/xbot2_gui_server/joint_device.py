import asyncio
from aiohttp import web
import json

import rospy
from std_srvs.srv import SetBool, Trigger
from xbot_msgs.msg import JointDeviceInfo
from xbot_msgs.srv import SetControlMask

from .server import ServerBase
from . import utils


class JointDeviceHandler:

    def __init__(self, srv: ServerBase, config=dict()) -> None:

        # config
        self.rate = config.get('rate', 10.0)

        # save server object, register our handlers
        self.srv = srv
        self.srv.schedule_task(self.run())
        self.srv.add_route('POST', '/joint/safety/set_filter_profile', self.set_filter_profile_handler, 'set_filter_profile')
        self.srv.add_route('POST', '/joint/safety/set_filter_active', self.set_filter_active_handler, 'set_filter_active')
        self.srv.add_route('POST', '/joint/safety/set_enabled', self.set_enabled_handler, 'set_enabled')

        # xbot2 joint device info
        self.jinfo_sub = rospy.Subscriber('xbotcore/joint_device_info', JointDeviceInfo, self.on_jdevinfo_recv, queue_size=1)
        self.msg = None

    
    @utils.handle_exceptions
    async def set_filter_profile_handler(self, request: web.Request):
        profile = request.rel_url.query['profile']
        set_profile = rospy.ServiceProxy(name=f'xbotcore/set_filter_profile_{profile}', service_class=Trigger)
        res = await utils.to_thread(set_profile)
        return web.Response(text=json.dumps({
            'success': res.success,
            'message': res.message,
            }))

    
    @utils.handle_exceptions
    async def set_filter_active_handler(self, request: web.Request):
        active = utils.str2bool(request.rel_url.query['active'])
        enable_joint_filter = rospy.ServiceProxy(name=f'xbotcore/enable_joint_filter', service_class=SetBool)
        res = await utils.to_thread(enable_joint_filter, active)
        return web.Response(text=json.dumps({
            'success': res.success,
            'message': res.message,
            }))

    
    @utils.handle_exceptions
    async def set_enabled_handler(self, request: web.Request):
        enabled = utils.str2bool(request.rel_url.query['enabled'])
        set_control_mask = rospy.ServiceProxy(name=f'xbotcore/joint_master/set_control_mask', service_class=SetControlMask)
        res = await utils.to_thread(set_control_mask, 255 if enabled else 0)
        return web.Response(text=json.dumps({
            'success': res.success,
            'message': res.message,
            }))


    async def run(self):

        while True:
            
            # periodic loop at 5 Hz
            await asyncio.sleep(1./self.rate)

            if not self.msg:
                continue
            
            # parse message to dict
            jdevinfo_msg = dict()
            jdevinfo_msg['type'] = 'joint_device_info'
            jdevinfo_msg['joint_active'] = self.msg.mask != 0
            jdevinfo_msg['filter_active'] = self.msg.filter_active
            jdevinfo_msg['filter_cutoff_hz'] = self.msg.filter_cutoff_hz
            
            # serialize msg to json
            msg_str = json.dumps(jdevinfo_msg)

            # send to all websocket clients
            await self.srv.ws_send_to_all(msg_str)
    
    
    def on_jdevinfo_recv(self, msg: JointDeviceInfo):
        self.msg = msg
        
