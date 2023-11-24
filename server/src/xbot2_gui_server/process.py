import asyncio
from aiohttp import web
import json
import re
import time

from .server import ServerBase
from .screen_session import Process
from . import utils


class ProcessHandler:

    procs = {}

    def __init__(self, srv: ServerBase, config=dict()) -> None:

        # config
        self.name = config['name']
        self.rate = config.get('rate', 10.)
        self.max_bw = config.get('max_bw_kbps', 1000)

        # create process
        self.proc = Process(name=self.name,
                            cmd=config['cmd'],
                            machine=config['machine'])

        # register to global list
        ProcessHandler.procs[self.name] = self.proc

        # set command line options from config
        for entry in config.get('args', []):
            self.proc.cmdline[entry['name']] = entry

        # save server object, register our handlers
        self.srv = srv
        self.srv.schedule_task(self.run())
        self.srv.schedule_task(self.proc_output_broadcaster(self.proc, stream_type='stdout'))
        self.srv.schedule_task(self.proc_output_broadcaster(self.proc, stream_type='stderr'))
        self.srv.add_route('GET', f'/process/get_list', 
                           self.process_get_list_handler, f'process_get_list_handler')
        self.srv.add_route('POST', f'/process/custom_command',
                           self.process_custom_command_handler, f'process_custom_command_handler')
        self.srv.add_route('PUT', f'/process/{self.name}/command/{{command}}',
                           self.process_command_handler, f'process_{self.name}_cmd_handler')
        self.srv.add_route('GET', f'/process/{self.name}/state',
                           self.process_state_handler, f'process_{self.name}_state_handler')

        # if running, attach
        self.srv.schedule_task(self.proc.attach())


    @utils.handle_exceptions
    async def process_custom_command_handler(self, request):
        
        body = await request.text()
        body = json.loads(body)
        
        cmd = body['command']
        machine = body['machine']
        timeout = float(body['timeout'])

        res = dict()
        res['success'] = False
        res['message'] = 'unknown failure'
        
        try:
            fut = Process.execute_command(machine=machine, cmd=cmd)
            ret, stdout, stderr = await asyncio.wait_for(fut=fut, timeout=timeout)
            res['success'] = True 
            res['message'] = f'command "{cmd}" returned {ret}'
            res['retcode'] = ret
            res['stdout'] = stdout.decode()
            res['stderr'] = stderr.decode()
        except asyncio.exceptions.TimeoutError:
            res['message'] = f'timeout = {timeout} s expired'
        
        return web.Response(text=json.dumps(res))
        



    @utils.handle_exceptions
    async def process_get_list_handler(self, request):
        
        # screen session data
        proc_data = list()
        for p in ProcessHandler.procs.values():
            proc_data.append ({
                'name': p.name,
                'status': 'Unknown',
                'cmdline': p.cmdline,
                'machine': p.hostname
            })

        return web.Response(text=json.dumps(proc_data))
    
    
    @utils.handle_exceptions
    async def process_command_handler(self, request: web.Request):

        cmd = request.match_info.get('command', None)

        res = dict()
        res['success'] = False
        res['message'] = 'unknown failure'

        if cmd == 'start':
            body = await request.text()
            body = json.loads(body)
            res['success'] = await self.proc.start(options=body.get('options', {}))
        elif cmd == 'stop':
            res['success'] = await self.proc.stop()
        elif cmd == 'kill':
            res['success'] = await self.proc.kill()
        else:
            res['message'] = f'{cmd} command not supported'
        if not res['success']:
            await self.srv.log(f'process_command_handler {cmd} failed ({self.name}): {res["message"]}', sev=2)

        return web.Response(text=json.dumps(res))

    
    @utils.handle_exceptions
    async def process_state_handler(self, request: web.Request):

        msg = {
            'success': True,
            'message': 'got status',
            'type': 'proc',
            'content': 'status',
            'name': self.proc.name,
            'status': await self.proc.status()
        }

        msg = json.dumps(msg)

        return web.Response(text=json.dumps(msg))

    
    async def run(self):

        while True:

            await asyncio.sleep(1./self.rate)

            msg = {
                'type': 'proc',
                'content': 'status',
                'name': self.proc.name,
                'status': await self.proc.status()
            }

            msg_str = json.dumps(msg)

            await self.srv.ws_send_to_all(msg_str)

    
    async def proc_output_broadcaster(self, p: Process, stream_type: str):

        ansi_escape = re.compile(r'\x1B(?:[@-Z\\-_]|\[[0-?]*[ -/]*[@-~])')

        throttle_print = True
        msg_size = 0
        prev_time = time.time()

        async_generator = getattr(p, f'read_{stream_type}')()

        while True:

            if not p.running:
                await asyncio.sleep(1./self.rate)
                continue

            line = await async_generator.asend(None)

            # line not empty, decode it and remove ansi colors
            line = ansi_escape.sub('', line.decode().strip())

            # throttle logic
            if time.time() - prev_time > 1.0:
                print(msg_size)
                prev_time = time.time()
                msg_size = 0
                throttle_print = True
            
            # compute msg size over a 1 sec window
            msg_size += len(line) + 80  # note: 80 bytes to account for json overhead

            # too much data: send once, then skip for the rest of the window duration
            if msg_size*8./1000. > self.max_bw:  # kbps -> Bps
                if throttle_print:
                    line = f'[{p.name}] process exceeding max output bandwith (max_bw = {self.max_bw} kbps) over a 1 sec window'
                    print(line)
                    throttle_print = False
                else:
                    continue

            msg = {
                'type': 'proc',
                'content': 'output',
                'name': p.name,
                'stdout': '',
                'stderr': '',
            }

            msg[stream_type] = line

            msg_str = json.dumps(msg)

            await self.srv.ws_send_to_all(msg_str)

