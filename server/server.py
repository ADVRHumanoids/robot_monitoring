import asyncio
import aiohttp
from typing import List
from aiohttp import web
import logging
import tempfile
import json


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


class Xbot2WebServer(ServerBase):

    def __init__(self) -> None:
        
        # web app
        self.app = web.Application()

        # event loop
        self.loop = asyncio.get_event_loop()

        # show all prints
        logging.basicConfig(level=logging.DEBUG)

        # websocket clients
        self.ws_clients = set()

        # ws callbacks
        self.ws_callbacks = list()

        # add routes
        routes = [
            ('GET', '/', self.root_handler, 'root'),
            ('GET', '/ws', self.websocket_handler, 'websocket'),
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

    
    def run_server(self, static='.', host='0.0.0.0', port=8080):
        
        # serve static files
        self.resource_dir = tempfile.mkdtemp('xbot2-gui-server')
        self.app.router.add_static('/resources', self.resource_dir, show_index=True)
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
        site = web.TCPSite(runner, host, port)
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
        print(f'new client connected (total is {len(self.ws_clients)})')

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

    