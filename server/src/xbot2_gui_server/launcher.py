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

class Launcher:

    def __init__(self, srv: ServerBase, config=dict()) -> None:
        
        launcher_cfg_path = config['launcher_config']
        if not os.path.isabs(launcher_cfg_path):
            launcher_cfg_path = os.path.join(os.path.dirname(srv.cfgpath), launcher_cfg_path)
        
        self.cfg = yaml.safe_load(open(config['launcher_config'], 'r'))

        self.rate = 3.333
        
        self.srv = srv

        self.srv.schedule_task(self.watch_all_processes())

        self.srv.schedule_task(self.run())

        self.srv.add_route('GET', f'/process/get_list', 
                           self.process_get_list_handler, f'process_launcher_get_list_handler')
        
        self.srv.add_route('POST', f'/process/custom_command',
                           self.process_custom_command_handler, f'process_custom_command_handler')
        
        self.srv.add_route('PUT', f'/process/{{name}}/command/{{command}}',
                           self.process_command_handler, f'process_launcher_cmd_handler')
        
        self.srv.add_route('GET', f'/process/{{name}}/state',
                           self.process_state_handler, f'process_launcher_state_handler')
        
        self.proc_stdout_bytes = 0
        self.proc_stdout_prev_time = 0
        self.proc_stdout_max_kbps = 1000
        self.proc_stdout_enabled = True

    @utils.handle_exceptions
    async def process_get_list_handler(self, request):
        
        # get process status
        # dict session -> process -> (pid, dead, exitstatus)
        status = await self.status()

        # screen session data
        proc_data = list()

        for p in self.get_process_names():

            variants = self.get_process_variants(p)

            proc_data.append ({
                'name': p,
                'status': status[p],
                'cmdline': variants,
                'machine': self.cfg[p].get('machine', 'local'),
                'visible': self.cfg[p].get('show_ui', True)
            })

        return web.Response(text=json.dumps(proc_data))
    
    
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
                    'type': 'proc',
                    'content': 'status',
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

        async def printer(l: str):

            msg = {
                'type': 'proc',
                'content': 'output',
                'name': process,
                'stdout': ansi_escape.sub('', l.strip()),
                'stderr': '',
            }

            msg_str = json.dumps(msg)

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
                    msg['stdout'] = f'[launcher] process exceeding max output bandwith (max_bw = {self.proc_stdout_max_kbps}) over a 1 sec window'
                    msg_str = json.dumps(msg)
                    self.proc_stdout_enabled = False
                else:
                    return

            await self.srv.ws_send_to_all(msg_str)

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

            msg_str = json.dumps(msg)

            await self.srv.ws_send_to_all(msg_str)

        
        return await exe.execute_process(process=process, 
                                         cfg=self.cfg,
                                         params=user_params,
                                         variants=user_variants,
                                         notify_event=on_launcher_event)
    
    async def kill(self, process, graceful=True):

        async def on_launcher_event(proc, text):

            msg = {
                'type': 'proc',
                'content': 'output',
                'name': 'launcher',
                'stdout': f'[{proc}] {text}',
                'stderr': '',
            }

            msg_str = json.dumps(msg)

            await self.srv.ws_send_to_all(msg_str)

        return await exe.kill(process=process, 
                              cfg=self.cfg, 
                              graceful=graceful,
                              notify_event=on_launcher_event)
        
    

    