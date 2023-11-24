import asyncio
import aiohttp
from typing import List
from aiohttp import web
import tempfile
import json
import os
from configparser import ConfigParser
from pathlib import Path
from pkg_resources import get_distribution
import os
import sys
import psutil
import logging

#import ssl

#ssl_context = ssl.create_default_context(ssl.Purpose.CLIENT_AUTH)
#ssl_context.load_default_certs()
#ssl_context.load_cert_chain(certfile='/home/alaurenzi/code/cert/localhost+1.pem',
#                            keyfile='/home/alaurenzi/code/cert/localhost+1-key.pem')

class ServerBase:

    async def ws_send_to_all(self, msg, clients=None):
        raise NotImplementedError

    def ws_clients(self) -> List[web.WebSocketResponse]:
        raise NotImplementedError

    def register_ws_coroutine(self, coroutine):
        raise NotImplementedError

    def add_route(self, method, path, handler, name):
        raise NotImplementedError

    def schedule_task(self, task):
        raise NotImplementedError
    
    async def log(self, txt, sev=0):
        raise NotImplementedError


class Xbot2WebServer(ServerBase):

    def __init__(self) -> None:
        
        # web app
        self.app = web.Application()

        # event loop
        self.loop = asyncio.get_event_loop()

        # websocket clients
        self.ws_clients = set()

        # ws callbacks
        self.ws_callbacks = list()
        self.register_ws_coroutine(self.handle_ping_msg)

        # add routes
        routes = [
            ('GET', '/', self.root_handler, 'root'),
            ('GET', '/ws', self.websocket_handler, 'websocket'),
            ('GET', '/version', self.version_handler, 'version'),
            ('POST', '/restart', self.restart_program, 'restart'),
        ]

        for route in routes:
            self.app.router.add_route(method=route[0], path=route[1], handler=route[2], name=route[3])

    
    def add_route(self, method, path, handler, name):
        try:
            self.app.router.add_route(method=method, path=path, handler=handler, name=name)
        except ValueError as e:
            pass

    
    def schedule_task(self, task):
        self.loop.create_task(task)

    
    def register_ws_coroutine(self, callback):
        self.ws_callbacks.append(callback)

    
    async def ws_send_to_all(self, msg, clients=None):

        if clients is None:
            clients = self.ws_clients

        # iterate over sockets (one per client)
        for ws in clients:
            
            try:
                await ws.send_str(msg)  
            except ConnectionResetError as e:
                pass
            except BaseException as e:
                print(f'error: {e}')


    async def log(self, txt, sev=0):
        
        msg = {
            'type': 'server_log',
            'txt': txt,
            'severity': sev
        }

        print(txt)

        await self.ws_send_to_all(json.dumps(msg))

    
    def run_server(self, static='.', host='0.0.0.0', port=8080):
        
        # serve static files
        self.resource_dir = tempfile.mkdtemp('xbot2-gui-server')
        self.app.router.add_static('/app', static, show_index=True)

        # run
        runner = web.AppRunner(self.app)

        # shorthand        
        loop = self.loop

        # heartbeat
        loop.create_task(self.heartbeat())

        # first execute server start routine
        loop.run_until_complete(self.start_http_server(runner, host, port))

        # then, loop forever
        loop.run_forever()

    
    async def start_http_server(self, runner, host, port):
        await runner.setup()
        site = web.TCPSite(runner, host, port)  #, ssl_context=ssl_context)
        await site.start()

      
    # heartbeat broadcaster
    async def heartbeat(self):
        
        """
        broadcast periodic heartbeat
        note: this also cleans up close websockets
        """

        # serialize msg to json
        msg_str = json.dumps(
            {
                'type': 'heartbeat'
            }
        )

        while True:

            # save disconnected sockets for later removal
            ws_to_remove = set()

            # iterate over sockets (one per client)
            for ws in self.ws_clients:
                
                try:
                    await ws.send_str(msg_str)  
                except ConnectionResetError as e:
                    ws_to_remove.add(ws)
                except BaseException as e:
                    print(f'error: {e}')
            
            for ws in ws_to_remove:
                self.ws_clients.remove(ws)
            
            # periodic loop at 10 Hz
            await asyncio.sleep(0.1)
    

    # root handler serves html for web app
    async def root_handler(self, request):
        return aiohttp.web.HTTPFound('/next_ui.html')

   
    # websocket handler
    async def websocket_handler(self, request):
        
        # connect
        ws = web.WebSocketResponse()
        await ws.prepare(request)

        # add new connection
        self.ws_clients.add(ws)
        await self.log(f'new client connected (total is {len(self.ws_clients)})')

        # start receiver task
        await self.websocket_receiver(ws)

        return ws

    
    async def websocket_receiver(self, ws):

        async for msg in ws:
            if msg.type == aiohttp.WSMsgType.TEXT:
                msg = json.loads(msg.data)
                for wcoro in self.ws_callbacks:
                    try:
                        await wcoro(msg, ws)
                    except BaseException as e:
                        print(f'[{wcoro.__func__.__qualname__}] error occurred: {e}')
            elif msg.type == aiohttp.WSMsgType.ERROR:
                print('ws connection closed with exception %s' %
                    ws.exception())
                break


    async def handle_ping_msg(self, msg, ws):
        if msg['type'] == 'ping':
            await self.ws_send_to_all(json.dumps(msg))


    async def version_handler(self, req: web.Request):
        config = ConfigParser()
        src_path = Path(os.path.abspath(__file__)).parent.parent
        root_path = src_path.parent
        cfg_path = os.path.join(root_path, 'setup.cfg')
        config.read(cfg_path)
        try:
            version = f"xbot2_gui_server {config['metadata']['version']}"
        except KeyError:
            version = get_distribution('hhcm-forest')

        return web.Response(text=json.dumps(
            {
                'success': True,
                'message': 'got version',
                'version': version
            }
        ))


    async def restart_program(self, req):
        """
        Restarts the current program, with file objects and descriptors
        cleanup
        """

        try:
            p = psutil.Process(os.getpid())
            for handler in p.get_open_files() + p.connections():
                os.close(handler.fd)
        except Exception as e:
            logging.error(e)

        python = sys.executable
        os.execl(python, python, *sys.argv)
