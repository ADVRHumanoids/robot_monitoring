import asyncio
from aiohttp import web
import json

import rospy
from std_srvs.srv import SetBool, Trigger
from cartesian_interface.srv import SetControlMode, SetControlModeRequest, GetTaskList, GetCartesianTaskInfo
from geometry_msgs.msg import TwistStamped

from .server import ServerBase
from . import utils


class CartesianHandler:

    def __init__(self, srv: ServerBase, config=dict()) -> None:

        # config
        self.rate = config.get('rate', 10.0)

        # save server object, register our handlers
        self.srv = srv
        self.srv.register_ws_coroutine(self.handle_ws_msg)
        self.srv.schedule_task(self.run())
        self.srv.add_route('GET', '/cartesian/get_task_list',
                           self.cartesian_get_task_list_handler,
                           'cartesian_get_task_list')
        self.srv.add_route('PUT', '/cartesian/{task_name}/set_control_mode',
                           self.cartesian_set_control_mode_handler,
                           'cartesian_set_control_mode')
        self.srv.add_route('GET', '/cartesian/{task_name}/get_cartesian_task_properties',
                           self.cartesian_get_cartesian_task_properties_handler,
                           'cartesian_get_cartesian_task_properties')

        # vel ref pub
        self.vref_pub = None

    
    @utils.handle_exceptions
    async def cartesian_get_task_list_handler(self, request):
        
        get_task_list = rospy.ServiceProxy('cartesian/get_task_list', GetTaskList)
        res = await utils.to_thread(get_task_list)
        return web.Response(text=json.dumps(
            {
                'success': True, 
                'message': 'ok',
                'names': res.names,
                'types': res.types,
            }
        ))


    @utils.handle_exceptions
    async def cartesian_get_cartesian_task_properties_handler(self, request):

        # we need the task name and ctrl mode
        task_name = request.match_info.get('task_name', None)
        if task_name is None:
            raise ValueError('no task name specified in url')

        # call service
        srv_name = f'cartesian/{task_name}/get_cartesian_task_properties'
        get_cartesian_task_properties = rospy.ServiceProxy(srv_name, GetCartesianTaskInfo)
        res = await utils.to_thread(get_cartesian_task_properties)

        return web.Response(text=json.dumps(
            {
                'success': True,
                'message': 'ok',
                'base_link': res.base_link,
                'distal_link': res.distal_link,
                'control_mode': res.control_mode,

            }
        ))


    async def run(self):
        pass
    

    @utils.handle_exceptions
    async def cartesian_set_control_mode_handler(self, request):

        # parse body into a python dict
        print(request)
        body = await request.text()
        print(body)
        body = json.loads(body)
        print(body)

        # we need the task name and ctrl mode
        task_name = request.match_info.get('task_name', None)
        if task_name is None:
            raise ValueError('no task name specified in url')
        control_mode = body['control_mode']

        # call service
        ctrl_mode_srv_name = rospy.resolve_name(f'cartesian/{task_name}/set_control_mode')
        ctrl_mode_srv = rospy.ServiceProxy(ctrl_mode_srv_name, service_class=SetControlMode)
        req = SetControlModeRequest()
        req.ctrl_mode = control_mode
        res = await utils.to_thread(ctrl_mode_srv, req)

        # respond to client
        return web.Response(text=json.dumps(
            {
                'success': res.success,
                'message': res.message,
            }
        ))


    async def handle_ws_msg(self, msg, ws):
        if msg['type'] == 'velocity_command':
            await self.handle_velocity_command(msg)


    async def handle_velocity_command(self, msg):

        task_name = msg['task_name']

        if(len(task_name) == 0):
            print('empty task name, skipping..')
            return

        topic_name = rospy.resolve_name(f'cartesian/{task_name}/velocity_reference')

        if self.vref_pub is None or self.vref_pub.resolved_name != topic_name:
            try:
                print(f'unregistering {self.vref_pub.name}')
            except:
                pass
            print(f'advertising {topic_name}')
            self.vref_pub = rospy.Publisher(topic_name, TwistStamped, queue_size=1)


        rosmsg = TwistStamped()
        rosmsg.header.stamp = rospy.Time.now()
        rosmsg.header.frame_id = task_name
        vref = msg['vref']
        rosmsg.twist.linear.x = vref[0]
        rosmsg.twist.linear.y = vref[1]
        rosmsg.twist.linear.z = vref[2]
        rosmsg.twist.angular.x = vref[3]
        rosmsg.twist.angular.y = vref[4]
        rosmsg.twist.angular.z = vref[5]

        self.vref_pub.publish(rosmsg)


