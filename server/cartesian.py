import asyncio
from aiohttp import web
import json

import rospy
from std_srvs.srv import SetBool, Trigger
from cartesian_interface.srv import SetControlMode, SetControlModeRequest, GetTaskList
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
        self.srv.add_route('GET', '/cartesian/get_task_list', self.cartesian_get_task_list_handler, 'cartesian_get_task_list')

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


    async def run(self):
        pass
    
    async def handle_ws_msg(self, msg, ws):
        if msg['type'] == 'velocity_command':
            await self.handle_velocity_command(msg)


    async def handle_velocity_command(self, msg):
        task_name = msg['task_name']
        topic_name = rospy.resolve_name(f'cartesian/{task_name}/velocity_reference')
        ctrl_mode_srv_name = rospy.resolve_name(f'cartesian/{task_name}/set_control_mode')

        if self.vref_pub is None or self.vref_pub.resolved_name != topic_name:
            try:
                print(f'unregistering {self.vref_pub.name}')
            except:
                pass
            print(f'waiting {ctrl_mode_srv_name}')
            ctrl_mode_srv = rospy.ServiceProxy(ctrl_mode_srv_name, service_class=SetControlMode)
            req = SetControlModeRequest()
            req.ctrl_mode = 'velocity'
            res = await utils.to_thread(ctrl_mode_srv, req)
            print(f'returned {res}')
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


