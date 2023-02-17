import asyncio
from aiohttp import web
import json

import rospy 
from xbot_msgs.msg import JointState, Fault
from urdf_parser_py import urdf as urdf_parser

from .server import ServerBase
from . import utils

class JointStateHandler:
    
    def __init__(self, srv: ServerBase, config=dict()) -> None:
        
        # save server object, register our handlers
        self.srv = srv
        self.srv.schedule_task(self.run())
        self.srv.add_route('GET', '/joint_states/info', self.get_joint_info_handler, 'get_joint_info')
        
        # joint state subscriber
        self.js_sub = rospy.Subscriber('xbotcore/joint_states', JointState, self.on_js_recv, queue_size=1)
        self.fault_sub = rospy.Subscriber('xbotcore/fault', Fault, self.on_fault_recv, queue_size=20)
        self.msg = None
        self.last_js_msg = None
        self.fault = None

        # config
        self.rate = config.get('rate', 60.0)
    
    
    @utils.handle_exceptions
    async def get_joint_info_handler(self, request: web.Request):

        joint_info = dict()

        if self.last_js_msg is not None:
            # convert to dict
            js_msg = JointStateHandler.js_msg_to_dict(self.last_js_msg)
            joint_info['message'] = 'ok'
            joint_info['success'] = True

        else:
            # message not available, report this as an error
            joint_info['message'] = 'joint states unavailable'
            joint_info['success'] = False
            return web.Response(text=json.dumps(joint_info))

        joint_info['jstate'] = js_msg
        joint_info['jnames'] = js_msg['name']

        # get urdf
        print('retrieving robot description..')
        urdf = rospy.get_param('xbotcore/robot_description', default='')
        if len(urdf) == 0:
            joint_info['message'] = 'unable to get robot description'
            joint_info['success'] = False
            return web.Response(text=json.dumps(joint_info))

        # parse urdf
        print('parsing urdf..')
        model = urdf_parser.Robot.from_xml_string(urdf)

        # read joint limits from urdf
        joint_info['qmin'] = list()
        joint_info['qmax'] = list()
        joint_info['vmax'] = list()
        joint_info['taumax'] = list()

        # todo: handle undefined limits
        for jn in js_msg['name']:
            joint = model.joint_map[jn]
            joint_info['qmin'].append(joint.limit.lower)
            joint_info['qmax'].append(joint.limit.upper)
            joint_info['vmax'].append(joint.limit.velocity)
            joint_info['taumax'].append(joint.limit.effort)

        print('done!')

        return web.Response(text=json.dumps(joint_info))


    async def run(self):

        while True:

            await asyncio.sleep(1./self.rate)

            if self.fault is not None:
                fault_msg = dict()
                fault_msg['type'] = 'joint_fault'
                fault_msg['name'] = self.fault.name
                fault_msg['fault'] = self.fault.fault
                self.fault = None
                await self.srv.ws_send_to_all(json.dumps(fault_msg))

            if self.msg is None:
                continue
            
            # convert to dict
            js_msg_to_send = JointStateHandler.js_msg_to_dict(self.msg)
            self.msg = None

            # serialize msg to json
            js_str = json.dumps(js_msg_to_send)

            # send to all connected clients
            await self.srv.ws_send_to_all(js_str)
            

    def on_js_recv(self, msg: JointState):
        self.msg = msg
        self.last_js_msg = msg


    def on_fault_recv(self, msg):
        self.fault = msg


    def js_msg_to_dict(msg: JointState):
        js_msg_dict = dict()
        js_msg_dict['type'] = 'joint_states'
        js_msg_dict['name'] = msg.name
        js_msg_dict['posRef'] = msg.position_reference
        js_msg_dict['motPos'] = msg.motor_position
        js_msg_dict['linkPos'] = msg.link_position
        js_msg_dict['torRef'] = msg.effort_reference
        js_msg_dict['tor'] = msg.effort
        js_msg_dict['velRef'] = msg.velocity_reference
        js_msg_dict['motVel'] = msg.motor_velocity
        js_msg_dict['linkVel'] = msg.link_velocity
        js_msg_dict['motorTemp'] = msg.temperature_motor
        js_msg_dict['driverTemp'] = msg.temperature_driver
        js_msg_dict['k'] = msg.stiffness
        js_msg_dict['d'] = msg.damping
        js_msg_dict['stamp'] = msg.header.stamp.to_sec()
        return js_msg_dict
