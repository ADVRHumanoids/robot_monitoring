import asyncio
from aiohttp import web
import json

from std_srvs.srv import SetBool
from xbot_msgs.srv import GetPluginList
from xbot_msgs.msg import Statistics2

from .server import ServerBase
from . import utils

# ros handle
from . import ros_utils
ros_handle : ros_utils.RosWrapper = ros_utils.ros_handle

class PluginHandler:

    def __init__(self, srv: ServerBase, config=dict()) -> None:
        
        # config
        self.rate = config.get('rate', 10.0)

        # save server object, register our handlers
        self.srv = srv
        self.srv.schedule_task(self.run())
        self.srv.add_route('PUT', '/plugin/{plugin_name}/command/{command}', self.plugin_cmd_handler, 'plugin_command')
        self.srv.add_route('GET', '/plugin/get_list', self.plugin_get_list_handler, 'plugin_get_list')

        # subscribe to plugin statistics
        self.pstat_sub = ros_handle.create_subscription(Statistics2, 
                                                        'xbotcore/statistics', 
                                                        self.on_pstat_recv, 
                                                        queue_size=1)
        self.msg = dict()

    
    @utils.handle_exceptions
    async def plugin_cmd_handler(self, request):

        res = dict()
        res['success'] = True
        
        plugin_name = request.match_info.get('plugin_name', None)
        command = request.match_info.get('command', None)
        
        if command not in ('start', 'stop'):
            res['message'] = f'invalid command {command}'
            res['success'] = False
            return web.Response(text=json.dumps(res))

        req_data = command == 'start'
        
        switch = ros_handle.create_client(SetBool, f'xbotcore/{plugin_name}/switch')
        res = await ros_handle.call(switch, timeout_sec=1.0, data=req_data)

        return web.Response(text=json.dumps({'success': res.success, 'message': res.message}))

    
    @utils.handle_exceptions
    async def plugin_get_list_handler(self, request):

        get_plugin_list = ros_handle.create_client(GetPluginList, 'xbotcore/get_plugin_list')
        plugin_list = await ros_handle.call(get_plugin_list, timeout_sec=1.0)

        return web.Response(text=json.dumps({
            'success': True, 
            'message': f'got plugin list',
            'plugins': plugin_list.plugins}))

    
    async def run(self):

        while True:
            
            # periodic loop at 5 Hz
            await asyncio.sleep(1./self.rate)

            if not self.msg:
                continue
            
            # parse message to dict
            ps_msg = dict()
            ps_msg['type'] = 'plugin_stats'
            
            # for each plugin, find the corresponding thread to get the expected period
            for ts in self.msg.task_stats:
                th = next(filter(lambda x: x.name == ts.thread, self.msg.thread_stats))
                ps_msg[ts.name] = {
                    'run_time': ts.run_time,
                    'expected_period': th.expected_period,
                    'state': ts.state,
                }

            # serialize msg to json
            msg_str = json.dumps(ps_msg)

            # send to all websocket clients
            await self.srv.udp_send_to_all(msg_str)
    
    
    def on_pstat_recv(self, msg: Statistics2):
        self.msg = msg
        
