import asyncio
from aiohttp import web
import json, yaml
import re
import logging
import time
import os
from typing import List

from concert_launcher import executor as exe
from concert_launcher import remote

from .server import ServerBase
from . import utils

from .proto import generic_pb2, process_output_pb2

class Launcher:

    def __init__(self, srv: ServerBase, config=dict()) -> None:
        
        launcher_cfg_path = config.get('launcher_config')
        
        self.cfg = None
        
        # no launcher config provided, use a default empty one
        # note: this allows dynamic process creation
        if launcher_cfg_path is None:
            self.cfg = {
                'context': {
                    'session': 'default',
                }
            }

        # path to a cache file to store additional process info (custom_commands, etc.)
        self.cache_path = config.get('cache_path', None)
        if self.cache_path is None:
            self.cache_path = os.path.expanduser('~/.cache/xbot2_gui_server/launcher_cache.yaml')
        
        # create required directory tree and cache file if it doesn't exist
        os.makedirs(os.path.dirname(self.cache_path), exist_ok=True)
        if not os.path.exists(self.cache_path):
            with open(self.cache_path, 'w') as f:
                yaml.dump({}, f)

        # launcher config provided, try to load it
        if self.cfg is None:
            if not os.path.isabs(launcher_cfg_path):
                launcher_cfg_path = os.path.join(os.path.dirname(srv.cfgpath), launcher_cfg_path)
            self.cfg = yaml.safe_load(open(launcher_cfg_path, 'r'))

        # add cached custom commands to config
        with open(self.cache_path, 'r') as f:
            cache = yaml.safe_load(f) or {}
        
        for process, data in cache.items():
            if process not in self.cfg:
                self.cfg[process] = data

        # status broadcast rate (Hz)
        self.rate = 3.333
        
        # save server 
        self.srv = srv

        # schedule tasks
        self.start_tasks()

        # add routes
        self.srv.add_route('GET', f'/process/get_list', 
                           self.process_get_list_handler, f'process_launcher_get_list_handler')
        
        self.srv.add_route('POST', f'/process/custom_command',
                           self.process_custom_command_handler, f'process_custom_command_handler')
        
        self.srv.add_route('PUT', f'/process/{{name}}/command/{{command}}',
                           self.process_command_handler, f'process_launcher_cmd_handler')
        
        self.srv.add_route('PUT', f'/process/add_custom_command',
                           self.process_add_custom_command_handler, f'process_add_custom_command_handler')
        
        self.srv.add_route('PUT', f'/process/delete_custom_command/{{name}}',
                           self.process_delete_custom_command_handler, f'process_delete_custom_command_handler')
        
        self.srv.add_route('GET', f'/process/{{name}}/state',
                           self.process_state_handler, f'process_launcher_state_handler')
        
        # init stdout throttling vars
        self.proc_stdout_bytes = 0
        self.proc_stdout_prev_time = 0
        self.proc_stdout_max_kbps = 1000
        self.proc_stdout_enabled = True


    async def stop_tasks(self):
        tasks_to_cancel = [self.watch_all_proc_task, self.run_task]
        tasks_to_cancel = [t for t in tasks_to_cancel if t is not None and not t.done()]
        for task in tasks_to_cancel:
            task.cancel()
        await asyncio.gather(*tasks_to_cancel, return_exceptions=True)


    def start_tasks(self):
        self.watch_all_proc_task = self.srv.schedule_task(self.watch_all_processes())
        self.run_task = self.srv.schedule_task(self.run())


    @utils.handle_exceptions
    async def process_get_list_handler(self, request):
        
        # get process status
        # dict session -> process -> (pid, dead, exitstatus)
        status = await self.status()

        # screen session data
        proc_data = list()

        for p in self.get_process_names():

            variants = self.get_process_variants(p)
            # ``docker`` is the container field already consumed by the QML
            # launcher. Accept ``container`` as an alias in configuration so
            # the read-only web metadata remains useful with either spelling.
            container = self.cfg[p].get('docker', self.cfg[p].get('container', ''))

            proc_data.append ({
                'name': p,
                'status': status[p],
                'cmdline': variants,
                'category': self.cfg[p].get('category', 'none'),
                'cmd': self.cfg[p].get('cmd', ''),
                'docker': container,
                'machine': self.cfg[p].get('machine', 'local'),
                'visible': self.cfg[p].get('show_ui', True),
                'cmd': self.cfg[p]['cmd'],
                'docker': self.cfg[p].get('docker', ''),
                'is_custom': self.cfg[p].get('_is_custom', False),
            })

        return web.Response(text=json.dumps(proc_data))
    

    @utils.handle_exceptions
    async def process_add_custom_command_handler(self, request):
        body = await request.text()
        body = json.loads(body)

        # extract required fields
        process = body['process']
        cmd = body['cmd']
        docker = body.get('docker')
        show_ui = body.get('show_ui', False)
        machine = body['machine']
        edit = body.get('edit', False)
        previous_name = body.get('previous_name', process)

        if '@' not in machine:
            return web.json_response({'success': False, 'message': f'invalid machine {machine} (must be "user@host")'})

        if not edit and process in self.cfg.keys():
            return web.json_response({'success': False, 'message': f'process {process} already exists'})
        
        # remove previous name (we support renaming processes)
        if edit:
            del self.cfg[previous_name]
        
        # add custom command to config
        self.cfg[process] = {
            'cmd': cmd,
            'machine': machine,
            'show_ui': show_ui,
            '_is_custom': True,
        }

        if docker is not None and docker != '':
            self.cfg[process]['docker'] = docker

        # add custom command to cache file for persistence across server restarts
        with open(self.cache_path, 'r') as f:
            cache = yaml.safe_load(f) or {}

        cache[process] = self.cfg[process]

        with open(self.cache_path, 'w') as f:
            yaml.dump(cache, f)

        # restart tasks to pick up new config (note: this is required to update the process list and status)
        await self.stop_tasks()
        self.start_tasks()

        # return success
        return web.json_response({'success': True, 'message': f'custom command {cmd} added to process {process}'})
    

    @utils.handle_exceptions
    async def process_delete_custom_command_handler(self, request):
        
        process = request.match_info.get('name', None)

        if process not in self.cfg.keys():
            return web.json_response({'success': False, 'message': f'process {process} does not exist'})
        
        # cancel tasks using self.cfg 
        await self.stop_tasks()

        # remove custom command from config
        del self.cfg[process]

        # remove custom command from cache file for persistence across server restarts
        with open(self.cache_path, 'r') as f:
            cache = yaml.safe_load(f) or {}

        if process in cache:
            del cache[process]

        with open(self.cache_path, 'w') as f:
            yaml.dump(cache, f)

        self.start_tasks()

        return web.json_response({'success': True, 'message': f'custom command {process} deleted'})
    

    @utils.handle_exceptions
    async def process_state_handler(self, request: web.Request):

        process = request.match_info.get('name', None)

        msg = {
            'success': True,
            'message': 'got status',
            'type': 'proc',
            'content': 'status',
            'name': process,
            'status': (await self.status())[process]
        }

        msg = json.dumps(msg)

        return web.Response(text=json.dumps(msg))


    @utils.handle_exceptions
    async def process_command_handler(self, request: web.Request):

        process = request.match_info.get('name', None)
        cmd = request.match_info.get('command', None)

        res = dict()
        res['success'] = False
        res['message'] = 'unknown failure'

        if cmd == 'start':
            body = await request.text()
            user_params = {}
            user_variants = []
            if len(body) > 0:
                body = json.loads(body)
                user_params, user_variants = self.parse_start_options(process=process, 
                                                                      options=body['options'])
            user_params.update({'roslaunch': 'roslaunch'})
            res['success'] = await self.start(process=process, 
                                              user_params=user_params, 
                                              user_variants=user_variants)
        elif cmd == 'stop':
            res['success'] = await self.kill(process=process)
        elif cmd == 'kill':
            res['success'] = await self.kill(process=process, graceful=False)
        else:
            res['message'] = f'{cmd} command not supported'
        if not res['success']:
            await self.srv.log(f'process_command_handler {cmd} failed ({self.name}): {res["message"]}', sev=2)

        return web.Response(text=json.dumps(res))
    

    @utils.handle_exceptions
    async def process_custom_command_handler(self, request):
        
        body = await request.text()
        body = json.loads(body)
        
        cmd = body['command']
        machine = body['machine']
        if machine == 'local':
            ssh = None
        else:
            ssh = exe.connection_map[machine]
        timeout = float(body['timeout'])

        res = dict()
        res['success'] = False
        res['message'] = 'unknown failure'
        
        try:
            fut = remote.run_cmd(ssh, cmd, interactive=True, throw_on_failure=False)
            ret, stdout, stderr = await asyncio.wait_for(fut=fut, timeout=timeout)
            res['success'] = True 
            res['message'] = f'command "{cmd}" returned {ret}'
            res['retcode'] = ret
            res['stdout'] = stdout
            res['stderr'] = stderr
        except asyncio.exceptions.TimeoutError:
            res['message'] = f'timeout = {timeout} s expired'
        
        return web.Response(text=json.dumps(res))
        

    async def run(self):

        while True:

            await asyncio.sleep(1./self.rate)

            logging.disable(logging.INFO)
            status = await self.status()
            logging.disable(logging.NOTSET)

            for p in self.get_process_names():

                msg = {
                    'type': 'proc_status',
                    'name': p,
                    'status': status[p]
                }

                msg_str = json.dumps(msg)

                await self.srv.udp_send_to_all(msg_str)


    def get_process_names(self):
        return [k for k in self.cfg.keys() if k != 'context']
    

    def get_process_variants(self, proc):
        
        e = exe.ConfigParser(process=proc, cfg=self.cfg, level=0)
        
        variants_dict = {}

        for v in e.variants:
            v: exe.Variant = v
            ventry = {}
            ventry['name'] = v.name
            ventry['default'] = 0
            ventry['help'] = 'TBD'
            ventry['type'] = 'combo' if len(v.choices) > 1 else 'check'
            ventry['options'] = ['Default'] + v.choices
            variants_dict[v.name] = ventry

        return variants_dict


    def create_proc_printer(self, process):

        ansi_escape = re.compile(r'\x1B(?:[@-Z\\-_]|\[[0-?]*[ -/]*[@-~])')

        @utils.print_exceptions
        async def printer(l: str):
            
            pbmsg = generic_pb2.Message()
            pbmsg.process_output.name = process
            pbmsg.process_output.out = ansi_escape.sub('', l.strip())

            # throttle logic
            if time.time() - self.proc_stdout_prev_time > 1.0:
                self.proc_stdout_prev_time = time.time()
                self.proc_stdout_bytes = 0
                self.proc_stdout_enabled = True
            
            # compute msg size over a 1 sec window
            self.proc_stdout_bytes += len(l) + 80  # note: 80 bytes to account for json overhead

            # too much data: send once, then skip for the rest of the window duration
            if self.proc_stdout_bytes*8/1000 > self.proc_stdout_max_kbps:  # kbps -> Bps
                if self.proc_stdout_enabled:
                    pbmsg.process_output.out = f'[launcher] process exceeding max output bandwith (max_bw = {self.proc_stdout_max_kbps}) over a 1 sec window'
                    self.proc_stdout_enabled = False
                else:
                    return

            await self.srv.ws_send_to_all(pbmsg)

        return printer


    async def watch_all_processes(self):
        await exe.watch(process=None, 
                        cfg=self.cfg, 
                        printer_coro_factory=self.create_proc_printer,
                        num_lines=100)


    async def status(self):

        status = await exe.status(process=None, cfg=self.cfg, print_to_stdout=False)

        # translate to proc -> (pid, dead, status)
        proc_status = {}
        for s, sdict in status.items():
            proc_status.update(**sdict)

        ret = {}

        # translate to readable names
        for p in self.get_process_names():
            
            # parse status into a string
            if p in proc_status.keys():
                if proc_status[p]['run_pending']:
                    status = 'Waiting'
                elif proc_status[p]['kill_pending']:
                    status = 'Killing'
                elif proc_status[p]['dead']:
                    status = 'Stopped'
                    if proc_status[p]['exitstatus'] != 0:
                        status = 'Killed'
                else:
                    status = 'Running'
            else:
                status = 'Stopped'

            ret[p] = status

        return ret
    

    def parse_start_options(self, process, options: dict):

        user_params = {}
        user_variants = []

        vars = self.get_process_variants(proc=process)
    
        for k, v in options.items():
            if vars[k]['type'] == 'combo' and v != 'Default':
                user_variants.append(v)
            elif v:
                user_variants.append(k)

        return user_params, user_variants
    

    async def start(self, process, user_params={}, user_variants=[]):
        
        async def on_launcher_event(proc, text):

            msg = {
                'type': 'proc',
                'content': 'output',
                'name': 'launcher',
                'stdout': f'[{proc}] {text}',
                'stderr': '',
            }

            pbmsg = generic_pb2.Message()
            pbmsg.process_output.name = 'launcher'
            pbmsg.process_output.out = f'[{proc}] {text}'
            await self.srv.ws_send_to_all(pbmsg)
        
        return await exe.execute_process(process=process, 
                                         cfg=self.cfg,
                                         params=user_params,
                                         variants=user_variants,
                                         notify_event=on_launcher_event)
    
    async def kill(self, process, graceful=True):

        async def on_launcher_event(proc, text):
            pbmsg = generic_pb2.Message()
            pbmsg.process_output.name = 'launcher'
            pbmsg.process_output.out = f'[{proc}] {text}'
            await self.srv.ws_send_to_all(pbmsg)

        return await exe.kill(process=process, 
                              cfg=self.cfg, 
                              graceful=graceful,
                              notify_event=on_launcher_event)
        
    

    
