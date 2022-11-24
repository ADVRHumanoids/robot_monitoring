import asyncio
from aiohttp import web
import json
import re

from .server import ServerBase
from .screen_session import Process
from . import utils


class ProcessHandler:

    procs = {}

    def __init__(self, srv: ServerBase, config=dict()) -> None:

        # config
        self.name = config['name']
        self.rate = config.get('rate', 10.)

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
                           ProcessHandler.process_get_list_handler, f'process_get_list_handler')
        self.srv.add_route('PUT', f'/process/{self.name}/command/{{command}}',
                           self.process_command_handler, f'process_{self.name}_cmd_handler')
        self.srv.add_route('GET', f'/process/{self.name}/state',
                           self.process_state_handler, f'process_{self.name}_state_handler')

    
    async def process_get_list_handler(request):
        
        # screen session data
        proc_data = list()
        for p in ProcessHandler.procs.values():
            proc_data.append ({
                'name': p.name,
                'status': 'Unknown',
                'cmdline': p.cmdline
            })

        return web.Response(text=json.dumps(proc_data))
    
    
    @utils.handle_exceptions
    async def process_command_handler(self, request: web.Request):

        cmd = request.match_info.get('command', None)

        res = dict()
        res['success'] = True

        if cmd == 'start':
            body = await request.text()
            body = json.loads(body)
            await self.proc.start(options=body.get('options', {}))
        elif cmd == 'stop':
            await self.proc.stop()
        elif cmd == 'kill':
            res['success'] = False
            res['message'] = 'kill command not supported yet'
        else:
            res['success'] = False
            res['message'] = f'{cmd} command not supported'

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

        while True:

            if p.proc is None:
                await asyncio.sleep(1./self.rate)
                continue
            
            # await stream to return something
            line = await getattr(p.proc, stream_type).readline()

            # note: important! if line is empty, process is dead!
            # we must set proc to none, otherwise this loop runs
            # too fast and starves all other coroutines!
            if len(line) == 0:
                p.proc = None
                continue

            # line not empty, decode it and remove ansi colors
            line = ansi_escape.sub('', line.decode().strip())

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

