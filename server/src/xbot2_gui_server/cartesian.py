import asyncio
from aiohttp import web
import json

import rospy
from std_srvs.srv import SetBool, Trigger
from cartesian_interface.srv import SetControlMode, SetControlModeRequest, GetTaskList, GetCartesianTaskInfo
from geometry_msgs.msg import TwistStamped, Twist

from .server import ServerBase
from . import utils


class CartesianHandler:

    def __init__(self, srv: ServerBase, config=dict()) -> None:

        # config
        self.rate = config.get('rate', 10.0)
        self.cmd_vel_topics = config.get('cmd_vel_topics', [])

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
        
        # get cartesio tasks
        try:
            get_task_list = rospy.ServiceProxy('cartesian/get_task_list', GetTaskList)
            res = await utils.to_thread(get_task_list)
        except:
            res = GetTaskList._response_class()

         # get topic names from ros master
        for tname in self.cmd_vel_topics:
            res.names.append(tname)
            res.types.append('SimpleTopic')

        print('DIODIOD', res.names, res.types)

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
        
        simple_topic_prefix = '[simple topic] '
        if task_name.startswith(simple_topic_prefix):
            topic_name = task_name[len(simple_topic_prefix):]
            TopicType = Twist
        else:
            topic_name = rospy.resolve_name(f'cartesian/{task_name}/velocity_reference')
            TopicType = TwistStamped

        if self.vref_pub is None or self.vref_pub.resolved_name != topic_name:
            try:
                print(f'unregistering {self.vref_pub.name}')
            except:
                pass
            print(f'advertising {topic_name}')
            self.vref_pub = rospy.Publisher(topic_name, TopicType, queue_size=1)


        rosmsg = TopicType()

        if TopicType is TwistStamped:
            rosmsg.header.stamp = rospy.Time.now()
            rosmsg.header.frame_id = task_name
            twist = rosmsg.twist
        else:
            twist = rosmsg

        vref = msg['vref']
        twist.linear.x = vref[0]
        twist.linear.y = vref[1]
        twist.linear.z = vref[2]
        twist.angular.x = vref[3]
        twist.angular.y = vref[4]
        twist.angular.z = vref[5]

        self.vref_pub.publish(rosmsg)


