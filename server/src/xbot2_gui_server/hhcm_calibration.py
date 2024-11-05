import asyncio
from aiohttp import web
import json
import yaml
import os

import rospy
from std_srvs.srv import SetBool, Trigger
from std_msgs.msg import String
from geometry_msgs.msg import TwistStamped, Twist
from xbot_msgs.msg import Statistics2
from xbot_msgs.srv import GetParameterInfo, SetString

from .server import ServerBase
from . import utils
from . import launcher

import subprocess


class HhcmCalibrationHandler:

    def __init__(self, srv: ServerBase, config=dict()) -> None:

        self.requested_pages = []

        self.srv = srv

        self.data_dir = subprocess.check_output(f"echo {config['data_dir']}", shell=True).decode().strip()

        self.motor_properties_file = subprocess.check_output(f"echo {config['motor_properties']}", shell=True).decode().strip()

        self.srv.add_route('GET', '/hhcm_calibration/properties',
                           self.hhcm_calibration_properties,
                           'hhcm_calibration_properties')

        self.srv.add_route('POST', '/hhcm_calibration/configure',
                           self.hhcm_calibration_configure,
                           'hhcm_calibration_configure')

        # self.srv.add_route('POST', '/parameters/set_value',
        #                    self.parameters_set_value,
        #                    'parameters_set_value')

        # # subscribe to stats
        # self.get_info = rospy.ServiceProxy('xbotcore/get_parameter_info', GetParameterInfo)
        # self.set_parameters = rospy.ServiceProxy('xbotcore/set_parameters', SetString)

    
    @utils.handle_exceptions
    async def hhcm_calibration_properties(self, request):

        motor_propertes = yaml.safe_load(open(self.motor_properties_file, 'r'))

        return web.json_response(
            {'success': True, 
             'message': 'all good here', 
             'data': motor_propertes}
             )
    
    @utils.handle_exceptions
    async def hhcm_calibration_configure(self, request: web.Request):
        
        # set parameters from body
        set_parameters = rospy.ServiceProxy('xbotcore/set_parameters', SetString)
        await utils.to_thread(set_parameters.wait_for_service, timeout=1)   

        body = await request.text()
        body = yaml.safe_load(body)
        print(body)

        log_file = os.path.join(self.data_dir, body['log_file'].replace('.', ''))
        freq_min = body['freq_min']
        freq_max = body['freq_max']

        params = {
            '/trajectory/log_file': log_file,
            '/trajectory/enable_log': True,
            '/trajectory/stop_time': 60.0,
            '/trajectory/j_motor/period': 20.0,
            '/trajectory/j_motor/freq_min': freq_min,
            '/trajectory/j_motor/freq_max': freq_max
        }

        res = await utils.to_thread(set_parameters, request=yaml.safe_dump(params))

        # TODO get motor identifier from SDO

        return web.json_response(
            {
                'success': res.success, 
                'message': res.message, 
            })
