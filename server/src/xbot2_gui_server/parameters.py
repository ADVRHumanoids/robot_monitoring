import asyncio
from aiohttp import web
import json
import yaml

import rospy
from std_srvs.srv import SetBool, Trigger
from std_msgs.msg import String
from geometry_msgs.msg import TwistStamped, Twist
from xbot_msgs.msg import Statistics2
from xbot_msgs.srv import GetParameterInfo, SetString

from .server import ServerBase
from . import utils
from . import launcher


class ParameterHandler:

    def __init__(self, srv: ServerBase, config=dict()) -> None:

        self.requested_pages = ['Parameters']

        self.rate = config.get('rate', 10.0)

        self.srv = srv

        self.srv.schedule_task(self.run())

        self.srv.add_route('GET', '/parameters/info',
                           self.parameters_get_info,
                           'parameters_get_info')

        # subscribe to stats
        self.get_info = rospy.ServiceProxy('xbotcore/get_parameter_info', GetParameterInfo)

    
    @utils.handle_exceptions
    async def parameters_get_info(self, request):

        response = await utils.to_thread(self.get_info, name=[], tunable_only=True)

        print(response)

        return web.json_response(
            {
                'name': response.name,
                'value': [json.dumps(yaml.safe_load(v)) for v in response.value],
                'type': response.type,
                'descriptor': [json.dumps(yaml.safe_load(v)) for v in response.descriptor]
            }
        )


    @utils.handle_exceptions
    async def dashboard_robot_switch(self, request: web.Request):
        
        # we need the command name (start or stop)
        command = request.match_info.get('command', None)

        l : launcher.Launcher = self.get_launcher()

        if command == 'start':
            
            # start ecat
            await self.send_status('starting robot 1/3 (ecat)')
            
            if not await l.start(process='ecat'):
                raise RuntimeError('could not start ecat')
            
            # check slaves
            await self.send_status('starting robot 2/3 (ecat_slave_check)')

            if not await l.start(process='wait_ecat_online'):
                raise RuntimeError('could not find slaves')
            
            await self.send_status('starting robot 3/3 (xbot2)')

            # start xbot2
            if not await l.start(process='xbot2',
                          user_variants=['ec_imp']):
                raise RuntimeError('could not start xbot2')
            
            # wait alive 
            while not self.xbot2_alive:
                await asyncio.sleep(0.1)
            
            self.active_state = 'ready'
            
        elif command == 'stop':

            # check xbot2 is down
            sdict = await l.status()

            if sdict.get('xbot2', '') == 'Running':
                raise RuntimeError('xbot2 is up: have you pressed the emergency button?') 

            # kill ecat and hope for the best
            await self.send_status('stopping robot (ecat)')

            if not await l.kill(process='ecat', graceful=True):
                raise RuntimeError('could not stop ecat')
            
            self.active_state = 'inactive'
            
        else:

            raise KeyError(f'bad command {command}')
                
        
        return web.Response(text=json.dumps(
            {
                'success': True,
                'message': 'ok',
            }
        ))


    @utils.handle_exceptions
    async def dashboard_start(self, request: web.Request):

        # check xbot2 alive
        if self.stats is None:
            raise RuntimeError('xbot2 is not alive')

        # we need the task name and ctrl mode
        task_name = request.match_info.get('task_name', None)
        
        if task_name is None:
            raise ValueError('no task name specified in url')
        
        task = self.states[task_name]

        # stop all plugins
        all_plugins = [t.name for t in self.stats.task_stats if t.name in self.all_plugins]
        plugins_to_start = plugins = task['plugin']
        plugins_to_stop = [p for p in all_plugins if p not in plugins_to_start]

        for p in plugins_to_stop:
            await self.send_status(f'stopping plugin {p}')
            await self.plugin_switch(p, 0)

        for p in plugins_to_stop:
            await self.send_status(f'waiting for plugin {p} to stop...')
            await asyncio.wait_for(self.wait_for_state(p, ['Stopped', 'Initialized']),
                                   timeout=10.0)

        # start required plugins
        for p in plugins_to_start:
            await self.send_status(f'starting plugin {p}')
            await self.plugin_switch(p, 1)
        for p in plugins_to_start:
            await self.send_status(f'waiting for plugin {p} to start...')
            await self.wait_for_state(p, 'Running')

        # kill all active processes
        process_to_start = task['process']
        process_to_stop = [p for p in self.all_processes if p not in process_to_start]
        for p in process_to_stop:
            await self.send_status(f'killing process {p}')
            l : launcher.Launcher = self.get_launcher()
            await asyncio.wait_for(l.kill(process=p, graceful=True),
                                   timeout=30)
            
        # start required processes
        for p in process_to_start:
            await self.send_status(f'starting process {p}')
            l : launcher.Launcher = self.get_launcher()
            await asyncio.wait_for(l.start(process=p),
                                   timeout=30)
            

        await self.send_status(f'done')
            
        self.active_state = task_name

        return web.Response(text=json.dumps(
            {
                'success': True,
                'message': 'ok',
            }
        ))
    

    async def plugin_switch(self, plugin_name, switch_flag):
        switch = rospy.ServiceProxy(f'xbotcore/{plugin_name}/switch', service_class=SetBool)
        await utils.to_thread(switch.wait_for_service, timeout=1.0)
        res = await utils.to_thread(switch, switch_flag)
        return res.success


    async def run(self):

        while True:

            # if not self.xbot2_alive:
            #     self.active_state = 'inactive'
            # elif self.active_state == 'inactive':
            #    self.active_state = 'ready'

            # self.xbot2_alive = False

            await self.srv.ws_send_to_all(
                {
                    'type': 'dashboard_msg',
                    'active_state': self.active_state
                }
            )

            await asyncio.sleep(0.666)


    def on_stats_recv(self, msg):
        self.stats = msg
        self.xbot2_alive = True
        


    async def wait_for_state(self, plugin_name, state):

        if isinstance(state, str):
            state = [state]

        idx = -1
        
        for i, t in enumerate(self.stats.task_stats):
            if t.name in plugin_name:
                idx = i
                break
        
        if idx == -1:
            raise KeyError(plugin_name)
        
        while self.stats.task_stats[idx].state not in state:
            await asyncio.sleep(0.1)
    

    def get_launcher(self):
        if self.launcher is not None:
            return self.launcher
        for e in self.srv.extensions:
            if isinstance(e, launcher.Launcher):
                self.launcher = e 
                return e
        raise ValueError('launcher unavailable')
    

    async def send_status(self, txt):
        print(txt)
        await self.srv.ws_send_to_all(
            dict(type='dashboard_msg',
                 msg=txt)
        )
