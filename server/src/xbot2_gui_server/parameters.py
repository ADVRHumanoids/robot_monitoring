import asyncio
from aiohttp import web
import json
import yaml

from std_srvs.srv import SetBool, Trigger
from std_msgs.msg import String
from geometry_msgs.msg import TwistStamped, Twist
from xbot_msgs.msg import Statistics2
from xbot_msgs.srv import GetParameterInfo, SetString

from .server import ServerBase
from . import utils
from . import launcher

# ros handle
from . import ros_utils
ros_handle : ros_utils.RosWrapper = ros_utils.ros_handle

class ParameterHandler:

    def __init__(self, srv: ServerBase, config=dict()) -> None:

        self.requested_pages = ['Parameters']

        self.srv = srv

        self.srv.add_route('GET', '/parameters/info',
                           self.parameters_get_info,
                           'parameters_get_info')

        self.srv.add_route('POST', '/parameters/set_value',
                           self.parameters_set_value,
                           'parameters_set_value')

        # subscribe to stats
        self.get_info = ros_handle.create_client(GetParameterInfo, 'xbotcore/get_parameter_info')
        self.set_parameters = ros_handle.create_client(SetString, 'xbotcore/set_parameter_values')

    
    @utils.handle_exceptions
    async def parameters_get_info(self, request):

        response = await ros_handle.call(self.get_info, timeout_sec=1.0, name=[], tunable_only=True)

        return web.json_response(
            {
                'name': response.name,
                'value': [json.dumps(yaml.safe_load(v)) for v in response.value],
                'type': response.type,
                'descriptor': [json.dumps(yaml.safe_load(v)) for v in response.descriptor]
            }
        )
    
    @utils.handle_exceptions
    async def parameters_set_value(self, request: web.Request):

        body = await request.text()

        print('setting parameters to', body)

        res = await ros_handle.call(self.set_parameters, timeout_sec=1.0, request=body)

        return web.json_response(
            {
                'success': res.success,
                'message': res.message
            }
        )
