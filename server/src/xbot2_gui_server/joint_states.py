import asyncio
from aiohttp import web
import json
import functools
import base64
import math
import time


import rospy 
from xbot_msgs.msg import JointState, Fault, JointCommand, CustomState
from std_msgs.msg import Float32
from urdf_parser_py import urdf as urdf_parser

from .server import ServerBase
from . import utils

# from .proto import joint_states_pb2

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
        self.srv.add_route('GET', '/joint_states/info', self.get_joint_info_handler, 'get_joint_info')
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

    
    @utils.handle_exceptions
    async def get_urdf_handler(self, request: web.Request):
        print('retrieving robot description..')
        urdf = rospy.get_param('xbotcore/robot_description', default='')
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
            
            # convert to dict
            js_msg_to_send = self.js_msg_to_dict(self.msg)

            # test: avoid sending names to save bw
            del js_msg_to_send['name']

            # add pow data and seq id
            js_msg_to_send['vbatt'] = self.vbatt
            js_msg_to_send['iload'] = self.iload
            js_msg_to_send['seq'] = self.js_seq
            self.js_seq += 1

            # serialize msg to json
            js_str = json.dumps(js_msg_to_send)      

            # send to all connected clients
            await self.srv.udp_send_to_all(js_str)

            # clear to avoid sending duplicates
            self.msg = None
            for v in self.aux_map.values():
                v.clear()

    
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
            
            qf = float(req.rel_url.query['qref'])
            trj_time = float(req.rel_url.query['time'])
            joint_name = req.match_info['joint_name']

            time = rospy.Time.now()
            t0 = time
            dt = 0.01
            jidx = self.last_js_msg.name.index(joint_name)
            q0 = self.last_js_msg.position_reference[jidx]
            
            print(f'commanding joint {joint_name} from q0 = {q0} to qf = {qf} in {trj_time} s')
            
            while time.to_sec() <= t0.to_sec() + trj_time \
                and not self.cmd_should_stop:

                tau = (time.to_sec() - t0.to_sec())/trj_time
                alpha = ((6*tau - 15)*tau + 10)*tau**3
                qref = q0*(1 - alpha) + qf*alpha
                msg = JointCommand()
                msg.name = [joint_name]
                msg.ctrl_mode = [1]
                msg.position = [qref]
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
