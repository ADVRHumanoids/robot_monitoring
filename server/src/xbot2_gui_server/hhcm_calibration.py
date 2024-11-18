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
from std_msgs.msg import Float32
from .dashboard import DashboardHandler

from .server import ServerBase
from . import utils
from . import launcher

import subprocess


class HhcmCalibrationHandler:

    def __init__(self, srv: ServerBase, config=dict()) -> None:

        self.requested_pages = []

        self.srv = srv

        srv.schedule_task(self.run())

        self.data_dir = subprocess.check_output(f"echo {config['data_dir']}", shell=True).decode().strip()

        self.upload_dir = subprocess.check_output(f"echo {config['upload_dir']}", shell=True).decode().strip()

        self.motor_properties_file = subprocess.check_output(f"echo {config['motor_properties']}", shell=True).decode().strip()

        self.srv.add_route('GET', '/hhcm_calibration/properties',
                           self.hhcm_calibration_properties,
                           'hhcm_calibration_properties')

        self.srv.add_route('POST', '/hhcm_calibration/configure',
                           self.hhcm_calibration_configure,
                           'hhcm_calibration_configure')

        self.srv.add_route('POST', '/hhcm_calibration/start',
                           self.hhcm_calibration_start_trj,
                           'hhcm_calibration_start_trj')

        self.srv.add_route('POST', '/hhcm_calibration/upload',
                           self.hhcm_calibration_upload,
                           'hhcm_calibration_upload')

        self.srv.add_route('POST', '/hhcm_calibration/calibrate',
                           self.hhcm_calibration_calibrate,
                           'hhcm_calibration_calibrate')
        
        self.progress_sub = rospy.Subscriber('/trajectory/progress', Float32, self.progress_recv)
        self.progress = None

        # self.srv.add_route('POST', '/parameters/set_value',
        #                    self.parameters_set_value,
        #                    'parameters_set_value')

        # # subscribe to stats
        # self.get_info = rospy.ServiceProxy('xbotcore/get_parameter_info', GetParameterInfo)
        # self.set_parameters = rospy.ServiceProxy('xbotcore/set_parameters', SetString)

    def progress_recv(self, msg: Float32):
        self.progress = msg.data


    async def run(self):

        while True:
            if self.progress is not None:
                await self.srv.ws_send_to_all({'type': 'hhcm_calib', 'progress': self.progress})
                self.progress = None 
            await asyncio.sleep(0.666)

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

        body_txt = await request.text()
        body = yaml.safe_load(body_txt)
        print(body)

        date_time = body['trj']['date_time'].replace('/', '_').replace(' ', '__').replace(',', '').replace(':', '_')
        
        trj_name = body['trj']['name']
        motor_type = body['motor_type']
        load_mass = body['load_mass']
        
        load_radius = body['load_radius']
        test_run = body['trj']['test_run']
        sub_dir = 'test' if test_run else 'calibration'
        dir_name = f'{self.data_dir}/{sub_dir}/{motor_type}/{date_time}'
        print(dir_name)
        os.makedirs(dir_name, exist_ok=True)
        with open(dir_name + '/DATA.yaml', 'a') as f:
            f.write(body_txt)
            f.write('\n')
        log_file = f'{dir_name}/{motor_type}_{trj_name}_{load_mass}M_{load_radius}R'.replace('.', '_')
        
        freq_min = body['trj']['omega_min']/6.28
        freq_max = body['trj']['omega_max']/6.28
        amplitude = body['trj']['amplitude']

        params = {
            '/trajectory/log_file': log_file,
            '/trajectory/enable_log': True,
            '/trajectory/stop_time': 60.0,
            '/trajectory/j_motor/period': 20.0,
            '/trajectory/j_motor/freq_min': freq_min,
            '/trajectory/j_motor/freq_max': freq_max,
            '/trajectory/j_motor/amplitude': amplitude
        }

        res = await utils.to_thread(set_parameters, request=yaml.safe_dump(params))

        # TODO get motor identifier from SDO

        return web.json_response(
            {
                'success': res.success, 
                'message': res.message, 
                'data_dir': dir_name
            })


    @utils.handle_exceptions
    async def hhcm_calibration_start_trj(self, request: web.Request):

        dash: DashboardHandler = None 
        for e in self.srv.extensions:
            if isinstance(e, DashboardHandler):
                dash = e 
        
        print('start homing')
        #await self.srv.ws_send_to_all({'type': 'hhcm_calib', 'progress': self.progress})
        await dash.plugin_switch('homing', switch_flag=1)

        await asyncio.sleep(1.0)

        print('wait homing')
        await dash.wait_for_state('homing', ['Stopped'])

        print('start trajectory')
        await dash.plugin_switch('trajectory', switch_flag=1)

        return web.json_response(
            {
                'success': True, 
                'message': 'done homing and started trajectory', 
            })



    @utils.handle_exceptions
    async def hhcm_calibration_upload(self, request: web.Request):

        # copy to onedrive folder
        proc = await asyncio.create_subprocess_shell(cmd=f'cp -rn {self.data_dir} {self.upload_dir}',
                                               cwd=self.data_dir,
                                               stdout=asyncio.subprocess.PIPE,
                                               stderr=asyncio.subprocess.PIPE)
        stdout, stderr = await asyncio.wait_for(proc.communicate(), timeout=10.0)
        stdout, stderr = stdout.decode(), stderr.decode()
        retcode = proc.returncode

        if retcode != 0:
            raise RuntimeError(f'copy failed with retcode {retcode}, stderr = {stderr}')
        
        # call sync
        l : launcher.Launcher = None
        for e in self.srv.extensions:
            if isinstance(e, launcher.Launcher):
                l = e

        await l.start(process='onedrive')

        return web.json_response(
            {
                'success': True, 
                'message': 'started syncing with onedrive client', 
            })



    @utils.handle_exceptions
    async def hhcm_calibration_calibrate(self, request: web.Request):

        # get data dir from body
        body_txt = await request.text()
        body = yaml.safe_load(body_txt)
        data_dir = body['data_dir']

        print(data_dir)

        # run calibration
        proc = await asyncio.create_subprocess_shell(cmd='rosrun hhcm_actuator_calibration simple_calib.py *.mat',
                                               cwd=data_dir,
                                               stdout=asyncio.subprocess.PIPE,
                                               stderr=asyncio.subprocess.PIPE)
        stdout, stderr = await asyncio.wait_for(proc.communicate(), timeout=10.0)
        stdout, stderr = stdout.decode(), stderr.decode()
        retcode = proc.returncode

        print(stdout)

        if retcode != 0:
            raise RuntimeError(f'calibration failed with retcode {retcode}, stderr = {stderr}')

        return web.json_response(
            {
                'success': True, 
                'message': 'done calibration', 
                'calib_result': stdout,
                'stderr': stderr
            })
