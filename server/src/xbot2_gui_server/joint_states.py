import asyncio
from aiohttp import web
import json
import functools
import base64
import math
import time
import numpy as np


import rospy 
from sensor_msgs.msg import JointState as StdJointState
from xbot_msgs.msg import JointState, Fault, JointCommand, CustomState
from std_msgs.msg import Float32
from urdf_parser_py import urdf as urdf_parser

from .server import ServerBase
from . import utils

from .proto import jointstate_pb2, generic_pb2

## limit float precision in json serialization
class RoundingFloat(float):
    __repr__ = staticmethod(lambda x: format(x, '.4f'))

json.encoder.c_make_encoder = None
if hasattr(json.encoder, 'FLOAT_REPR'):
    # Python 2
    json.encoder.FLOAT_REPR = RoundingFloat.__repr__
else:
    # Python 3
    json.encoder.float = RoundingFloat


class JointStateHandler:
    
    def __init__(self, srv: ServerBase, config=dict()) -> None:
        
        # save server object, register our handlers
        self.srv = srv
        self.srv.schedule_task(self.run())
        self.srv.register_ws_coroutine(self.handle_ws_msg)
        self.srv.add_route('GET', '/joint_states/info', self.get_joint_info_handler, 'get_joint_info')
        self.srv.add_route('GET', '/joint_states/grippers', self.get_grippers_handler, 'get_grippers')
        self.srv.add_route('GET', '/joint_states/urdf', self.get_urdf_handler, 'get_urdf')
        self.srv.add_route('GET', '/joint_states/connected', self.robot_connected_handler, 'get_connected')
        self.srv.add_route('PUT', '/joint_command/goto/{joint_name}', self.command_handler, 'command')
        self.srv.add_route('POST', '/joint_command/goto/stop', self.stop_handler, 'stop')
        
        # joint state subscriber
        self.js_sub = rospy.Subscriber('xbotcore/joint_states', JointState, self.on_js_recv, queue_size=1)
        self.fault_sub = rospy.Subscriber('xbotcore/fault', Fault, self.on_fault_recv, queue_size=20)
        self.msg = None
        self.last_js_msg = None
        self.fault = None
        self.js_seq = 0

        # temperatures
        self.mot_temp = None
        self.dri_temp = None
        self.mot_temp_sub = rospy.Subscriber('xbotcore/temperature_motor', CustomState, self.on_mot_temp_recv, queue_size=1)
        self.dri_temp_sub = rospy.Subscriber('xbotcore/temperature_driver', CustomState, self.on_dri_temp_recv, queue_size=1)

        # vbatt iload
        self.vbatt_sub = rospy.Subscriber('xbotcore/vbatt', Float32, self.on_vbatt_recv, queue_size=1)
        self.iload_sub = rospy.Subscriber('xbotcore/iload', Float32, self.on_iload_recv, queue_size=1)
        self.vbatt = 0
        self.iload = 0

        # aux
        self.aux_map = dict()
        self.aux_subs = []
        self.js_to_aux_id = []
                    
        # command publisher
        self.cmd_pub = rospy.Publisher('xbotcore/command', JointCommand, queue_size=1)
        self.cmd_busy = False
        self.cmd_guard = JointStateHandler.CommandGuard(self.command_acquire, self.command_release)
        self.cmd_should_stop = True

        # grippers
        self.gripper_state_sub = dict()
        self.gripper_cmd_pub = dict()
        self.gripper_state_msg = dict()

        # config
        self.rate = config.get('rate', 30.0)

    
    def on_aux_recv(self, msg: CustomState, aux: list):

        if len(self.js_to_aux_id) == 0 and self.last_js_msg is not None:
            
            for _, js_name in enumerate(self.last_js_msg.name):
                
                try:
                    self.js_to_aux_id.append( msg.name.index(js_name) )
                except ValueError:
                    self.js_to_aux_id.append(-1)

        
        if len(aux) != len(msg.value):
            aux.clear()
            aux.extend(msg.value)
            return 

        for i in range(len(msg.value)):
            if not math.isnan(msg.value[i]):
                aux[i] = msg.value[i]


    def on_gripper_state_recv(self, msg: StdJointState, gname):
        self.gripper_state_msg[gname] = msg


    @utils.handle_exceptions
    async def get_urdf_handler(self, request: web.Request):
        print('retrieving robot description..')
        urdf = rospy.get_param('xbotcore/robot_description', default='')
        urdf = urdf.replace('<texture/>', '')
        return web.Response(text=json.dumps({'urdf': urdf}))
    

    @utils.handle_exceptions
    async def robot_connected_handler(self, request: web.Request):
        self.msg = None
        for _ in range(10):
            if self.msg is not None:
                return web.Response(text=json.dumps({'response': True}))
            await asyncio.sleep(0.01)
        return web.Response(text=json.dumps({'response': False}))
        

    @utils.handle_exceptions
    async def get_joint_info_handler(self, request: web.Request):

        joint_info = dict()

        if self.last_js_msg is not None:
            # convert to dict
            js_msg = self.js_msg_to_dict(self.last_js_msg)
            joint_info['message'] = 'ok'
            joint_info['success'] = True

        else:
            # message not available, report this as an error
            joint_info['message'] = 'joint states unavailable'
            joint_info['success'] = False
            return web.Response(text=json.dumps(joint_info))

        joint_info['jstate'] = js_msg
        joint_info['jnames'] = js_msg['name']

        # aux
        self.aux_map = dict()
        self.aux_subs = []
        for tname, ttype in rospy.get_published_topics():
            tname: str = tname
            if ttype == 'xbot_msgs/CustomState' and 'aux/' in tname:
                aux_type_name = 'aux/' + tname[tname.find('aux/')+4:]
                self.aux_map[aux_type_name] = list()
                cb = functools.partial(self.on_aux_recv, aux=self.aux_map[aux_type_name])
                sub = rospy.Subscriber(tname, CustomState, cb, queue_size=10)
                self.aux_subs.append(sub)

        # get urdf
        print('retrieving robot description..')
        urdf = rospy.get_param('xbotcore/robot_description', default='')
        urdf = urdf.replace('<texture/>', '')
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


    @utils.handle_exceptions
    async def get_grippers_handler(self, req: web.Request):

        # get topic names from ros master
        topic_name_type_list = await utils.to_thread(rospy.get_published_topics)

        # filter /xbotcore/gripper/GRIPPERNAME/state
        gripper_names = set()
        for tname, ttype in topic_name_type_list:
            print(tname, ttype)
            if ttype != 'sensor_msgs/JointState':
                continue
            tokens = tname.strip('/').split('/')
            if len(tokens) == 4 and tokens[1] == 'gripper' and tokens[3] == 'state':
                gripper_names.add(tokens[2])

        # register topics
        for gname in gripper_names:
            state = f'xbotcore/gripper/{gname}/state'
            cmd = f'xbotcore/gripper/{gname}/command'
            self.gripper_state_msg[gname] = None
            self.gripper_state_sub[gname] = rospy.Subscriber(state, StdJointState, 
                self.on_gripper_state_recv, gname, queue_size=1)
            self.gripper_cmd_pub[gname] = rospy.Publisher(cmd, StdJointState, queue_size=1)
            print(f'connecting to gripper {gname}...')
            
        
        # reply
        res = {
            'success': True,
            'message': '',
            'gripper_names': sorted(list(gripper_names))
        }

        return web.Response(text=json.dumps(res))
            

    async def run(self):

        t0 = time.time()

        while True:
            
            # sync loop at given rate
            tnow = time.time()
            await asyncio.sleep(1./self.rate - (tnow - t0))
            t0 = tnow

            # broadcast fault
            if self.fault is not None:
                fault_msg = dict()
                fault_msg['type'] = 'joint_fault'
                fault_msg['name'] = self.fault.name
                fault_msg['fault'] = self.fault.fault
                self.fault = None
                await self.srv.ws_send_to_all(json.dumps(fault_msg))

            # check js received
            if self.msg is None:
                continue

            # pb js
            # TBD aux support
            try:
                msgpb = generic_pb2.Message()
                msgpb.jointstate.linkPos.extend(self.msg.link_position)
                msgpb.jointstate.motPos.extend(self.msg.motor_position)
                msgpb.jointstate.motVel.extend(self.msg.motor_velocity)
                msgpb.jointstate.velRef.extend(self.msg.velocity_reference)
                msgpb.jointstate.torRef.extend(self.msg.effort_reference)
                msgpb.jointstate.tor.extend(self.msg.effort)
                msgpb.jointstate.posRef.extend(self.msg.position_reference)
                msgpb.jointstate.k.extend(self.msg.stiffness)
                msgpb.jointstate.d.extend(self.msg.damping)
                msgpb.jointstate.motorTemp.extend(self.msg.temperature_motor)
                msgpb.jointstate.driverTemp.extend(self.msg.temperature_driver)
                msgpb.jointstate.vbatt = self.vbatt
                msgpb.jointstate.ibatt = self.iload
                await self.srv.udp_send_to_all(msgpb)
            except Exception as e:
                # print traceback  
                import traceback
                traceback.print_exc()
                

            # clear to avoid sending duplicates
            self.msg = None
            for v in self.aux_map.values():
                v.clear()

            # grippers
            for gname, gmsg in self.gripper_state_msg.items():
                if gmsg is None:
                    continue 
                gmsg = {
                    'type': 'gripper_state',
                    'q': gmsg.position[0],
                    'tau': gmsg.effort
                }
                await self.srv.ws_send_to_all(json.dumps(gmsg))
                self.gripper_state_msg[gname] = None

    
    def command_acquire(self):
        if self.cmd_busy:
            raise RuntimeError('joint command busy')
        self.cmd_busy = True

    def command_release(self):
        self.cmd_busy = False

    class CommandGuard:
        def __init__(self, acq, res) -> None:
            self.acquire = acq 
            self.release = res 
        def __enter__(self):
            self.acquire()
        def __exit__(self, *args):
            self.release()
    
    @utils.handle_exceptions
    async def command_handler(self, req: web.Request):

        with self.cmd_guard:

            self.cmd_should_stop = False
            
            qf = np.array(list(map(float, req.rel_url.query['qref'].split(';'))))
            trj_time = float(req.rel_url.query['time'])
            try:
                ctrl = req.rel_url.query['ctrl']
            except KeyError:
                ctrl = 'Position'
            joint_name = req.match_info['joint_name'].split(';')

            time = rospy.Time.now()
            t0 = time
            dt = 0.01
            jidx = [self.last_js_msg.name.index(jn) for jn in joint_name]

            if ctrl == 'Position':
                q0 = np.array(self.last_js_msg.position_reference)[jidx]
            elif ctrl == 'Stiffness':
                q0 = np.array(self.last_js_msg.stiffness)[jidx]
            elif ctrl == 'Damping':
                q0 = np.array(self.last_js_msg.damping)[jidx]
            else:
                raise ValueError(f'unknown control mode {ctrl}')
            
            print(f'commanding {ctrl} joint {joint_name} from q0 = {q0} to qf = {qf} in {trj_time} s')
            
            while time.to_sec() <= t0.to_sec() + trj_time \
                and not self.cmd_should_stop:

                tau = (time.to_sec() - t0.to_sec())/trj_time
                alpha = ((6*tau - 15)*tau + 10)*tau**3
                qref = q0*(1 - alpha) + qf*alpha
                msg = JointCommand()
                msg.name = joint_name
                if ctrl == 'Position':
                    msg.ctrl_mode = [1]*len(joint_name)
                    msg.position = qref.tolist()
                elif ctrl == 'Stiffness':
                    msg.ctrl_mode = [8]*len(joint_name)
                    msg.stiffness = qref.tolist()
                elif ctrl == 'Damping':
                    msg.ctrl_mode = [16]*len(joint_name)
                    msg.damping = qref.tolist()
                self.cmd_pub.publish(msg)
                await asyncio.sleep(dt)
                time = rospy.Time.now()

            if self.cmd_should_stop:
                print('trj stopped!')
                return web.json_response(
                    {
                        'success': True,
                        'message': f'stopped while commanding joint {joint_name} from q0 = {q0} to qf = {qf} in {trj_time} s'
                    }
                ) 
            else:
                print('trj done!')
                return web.json_response(
                    {
                        'success': True,
                        'message': f'commanded joint {joint_name} from q0 = {q0} to qf = {qf} in {trj_time} s'
                    }
                )

    
    @utils.handle_exceptions
    async def stop_handler(self, req: web.Request):
        self.cmd_should_stop = True 
        return web.Response(text='["ok"]')
            

    def on_js_recv(self, msg: JointState):
        self.msg = msg
        self.last_js_msg = msg


    def on_fault_recv(self, msg):
        self.fault = msg


    def on_vbatt_recv(self, msg):
        self.vbatt = msg.data


    def on_iload_recv(self, msg):
        self.iload = msg.data


    def on_mot_temp_recv(self, msg):
        self.mot_temp = msg


    def on_dri_temp_recv(self, msg):
        self.dri_temp = msg


    def js_msg_to_dict(self, msg: JointState):
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
        js_msg_dict['motorTemp'] = msg.temperature_motor if self.mot_temp is None else self.mot_temp.value
        js_msg_dict['driverTemp'] = msg.temperature_driver if self.dri_temp is None else self.dri_temp.value
        js_msg_dict['k'] = msg.stiffness
        js_msg_dict['d'] = msg.damping
        js_msg_dict['stamp'] = msg.header.stamp.to_sec()
        js_msg_dict['aux_types'] = []
        for k, v in self.aux_map.items():
            if len(v) > 0:
                js_msg_dict[k] = [None if ai < 0 or math.isnan(v[ai]) else v[ai] for ai in self.js_to_aux_id]
                js_msg_dict['aux_types'].append(k)
        return js_msg_dict


    async def handle_ws_msg(self, msg, proto, ws):

        if msg['type'] == 'joint_cmd':
            cmdmsg = JointCommand()
            cmdmsg.name = msg['joint_names']
            if msg['ctrl'] == 'Velocity':
                cmdmsg.ctrl_mode = [2]*len(cmdmsg.name)
                cmdmsg.velocity = msg['command']
            elif msg['ctrl'] == 'Effort':
                cmdmsg.ctrl_mode = [4]*len(cmdmsg.name)
                cmdmsg.effort = msg['command']
            else:
                raise ValueError(f'unknown control mode {msg["ctrl"]}')
            self.cmd_pub.publish(cmdmsg)

        if msg['type'] == 'gripper_cmd':
            cmdmsg = StdJointState()
            gname = msg['name']
            if msg['action'] == 'open':
                cmdmsg.position = [0.0]
            elif msg['action'] == 'close':
                cmdmsg.effort = [msg['effort']]
            self.gripper_cmd_pub[gname].publish(cmdmsg)
