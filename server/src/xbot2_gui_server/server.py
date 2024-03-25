import asyncio
import aiohttp
import asyncudp
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
import time
import subprocess
from subprocess import run
import ssl 
import netifaces as ni

from . import utils

def generate_mkcert_certificate(addresses):
    """ "Generates a https certificate for localhost and selected addresses. This
    requires that the mkcert utility is installed, and that a certificate
    authority key pair (rootCA-key.pem and rootCA.pem) has been generated. The
    certificates are written to /tmp, where the http server can find them
    ater on."""

    cert_file = tempfile.NamedTemporaryFile(
        mode="w+b", prefix="qtwasmserver-certificate-", suffix=".pem", delete=True
    )
    cert_key_file = tempfile.NamedTemporaryFile(
        mode="w+b", prefix="qtwasmserver-certificate-key-", suffix=".pem", delete=True
    )

    # check if mkcert is installed
    try:
        out = subprocess.check_output(["mkcert", "-CAROOT"])
        root_ca_path = out.decode("utf-8").strip()
        print(
            "Generating certificates with mkcert, using certificate authority files at:"
        )
        print(f"   {root_ca_path}       [from 'mkcert -CAROOT'] \n")
    except Exception as e:
        print("Warning: Unable to run mkcert. Will not start https server.")
        print(e)
        print(f"Install mkcert from github.com/FiloSottile/mkcert to fix this.\n")
        return False, None, None

    # generate certificates using mkcert
    addresses_string = f"localhost {' '.join(addresses)}"
    print("=== begin mkcert output ===\n")
    ret = run(
        f"mkcert -cert-file {cert_file.name} -key-file {cert_key_file.name} {addresses_string}",
        shell=True,
    )
    print("=== end mkcert output ===\n")
    has_certificate = ret.returncode == 0
    if not has_certificate:
        print(
            "Warning: mkcert is not installed or was unable to create a certificate. Will not start HTTPS server."
        )
    return has_certificate, cert_file, cert_key_file


class ServerBase:

    async def ws_send_to_all(self, msg, clients=None):
        raise NotImplementedError

    async def udp_send_to_all(self, msg, clients=None):
        raise NotImplementedError

    async def msg_send_to_all(self, msg, proto:str, clients=None):
        if proto == 'udp':
            return await self.udp_send_to_all(msg, clients)
        elif proto == 'ws':
            return await self.ws_send_to_all(msg, clients)
        else:
            raise ValueError(f'unsupported protocol "{proto}"')


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
        self.port = None

        # event loop
        self.loop = asyncio.get_event_loop()

        # websocket clients
        self.ws_clients = set()

        # ws callbacks
        self.ws_callbacks = list()
        self.register_ws_coroutine(self.handle_ping_msg)

        # udp local endpoint
        self.udp : asyncudp.Socket = None
        self.udp_clients = set()  # set of pairs (addr, port)
        self.udp_clients_timeout = dict()

        # add routes
        routes = [
            ('GET', '/', self.root_handler, 'root'),
            ('GET', '/ws', self.websocket_handler, 'websocket'),
            ('GET', '/udp', self.udp_handler, 'udp'),
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

        if len(clients) > 0 and isinstance(msg, dict):
            msg = json.dumps(msg)


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

    
    async def udp_send_to_all(self, msg: str, clients=None):

        if self.udp is None:
            return False
        
        if clients is None:
            clients = self.udp_clients

        for addr in clients:
            self.udp.sendto(msg.encode(), addr)

        return True

    
    def run_server(self, static='.', host='0.0.0.0', port=8080):
        
        # serve static files
        self.resource_dir = tempfile.mkdtemp('xbot2-gui-server')
        self.app.router.add_static('/app', static, show_index=True)

        # detect nic
        addresses = ["127.0.0.1"]
        addresses += [
            addr[ni.AF_INET][0]["addr"]
            for addr in map(ni.ifaddresses, ni.interfaces())
            if ni.AF_INET in addr
        ]
        addresses = list(set(addresses))  # deduplicate
        addresses.sort()

        # ssl
        has_certificate, cert_file, cert_key_file = generate_mkcert_certificate(addresses)
        if has_certificate:
            context = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
            context.load_cert_chain(cert_file.name, cert_key_file.name)
        else:
            context = None

        # run
        runner = web.AppRunner(self.app)

        # shorthand        
        loop = self.loop

        # heartbeat
        loop.create_task(self.heartbeat())

        # first execute server start routine
        loop.run_until_complete(self.start_http_server(runner, host, port, None))

        # save port
        self.port = port

        # run udp receiver
        self.schedule_task(self.udp_receiver())

        # then, loop forever
        loop.run_forever()

    
    async def start_http_server(self, runner, host, port, ssl_context):
        await runner.setup()
        print('starting web server')
        site = web.TCPSite(runner, host, port, ssl_context=ssl_context)
        await site.start()
        print('started web server')

      
    # heartbeat broadcaster
    async def heartbeat(self):
        
        """
        broadcast periodic heartbeat with server-relate lightweight data
        note: this also cleans up close websockets
        """

        # serialize msg to json
        msg_str = json.dumps(
            {
                'type': 'heartbeat',
            }
        )

        while True:

            # save disconnected sockets for later removal
            ws_to_remove = set()
            udp_to_remove = set()

            # check udp timeout
            for addr, timeout in self.udp_clients_timeout.items():
                if time.time() > timeout:
                    print(f'removing stale udp remote {addr}')
                    udp_to_remove.add(addr)

            # iterate over sockets (one per client)
            for ws in self.ws_clients:
                
                try:
                    await ws.send_str(msg_str)  
                except ConnectionResetError as e:
                    print(f'removing stale ws remote')
                    ws_to_remove.add(ws)
                except BaseException as e:
                    print(f'error: {e}')
            
            for ws in ws_to_remove:
                self.ws_clients.remove(ws)

            for addr in udp_to_remove:
                self.udp_clients.remove(addr)
                del self.udp_clients_timeout[addr]
            
            # periodic loop at 10 Hz
            await asyncio.sleep(0.666)
    

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
                        await wcoro(msg, 'ws', ws)
                    except BaseException as e:
                        print(f'[{wcoro.__func__.__qualname__}] error occurred: {e}')
            elif msg.type == aiohttp.WSMsgType.ERROR:
                print('ws connection closed with exception %s' %
                    ws.exception())
                break


    async def handle_ping_msg(self, msg, proto, ws):
        if msg['type'] == 'ping':
            await self.msg_send_to_all(json.dumps(msg), proto=proto, clients=[ws])

    
    async def create_udp_socket(self):
        port = self.port + 0
        while True:
            try:
                self.udp = await asyncudp.create_socket(local_addr=('0.0.0.0', port))
                print(f'udp socket bound to port {port}')
                return 
            except OSError as e:
                if e.errno == 98:  # address already in use
                    print(f'port {port} already in use, will try a different one')
                    port += 1  # retry
                else:
                    raise RuntimeError('create socket failed: ' + str(e))



    @utils.handle_exceptions
    async def udp_handler(self, req: web.Request):

        addr, port = self.udp.getsockname()

        return web.Response(text=json.dumps(
            {
                'success': True,
                'port': port
            }
        ))
    

    async def udp_receiver(self):
        
        # bind udp socket
        if self.udp is None:
            await self.create_udp_socket()
            addr, port = self.udp.getsockname()
            await self.log(f'server opened udp endpoint at {addr}:{port}')

        # receiver's infinite loop
        while True:

            try:
                
                # wait till message received
                data, addr = await self.udp.recvfrom()

                # decode to string
                msg = data.decode()

                # update timeout for this client
                self.udp_clients_timeout[addr] = time.time() + 10.0

                # handle udp discovery messages
                if msg == 'udp_discovery' and addr not in self.udp_clients:

                    self.udp_clients.add(addr)

                    await self.log(f'added udp client at {addr}, total is {len(self.udp_clients)}')

                # handle general messages
                if msg != 'udp_discovery':

                    msg = json.loads(msg)

                    # invoke registered callbacks
                    for wcoro in self.ws_callbacks:
                        try:
                            await wcoro(msg, 'udp', addr)
                        except BaseException as e:
                            print(f'[{wcoro.__func__.__qualname__}] error occurred: {e}')


            except asyncudp.ClosedError as e:

                print(e)



    @utils.handle_exceptions
    async def version_handler(self, req: web.Request):
        config = ConfigParser()
        src_path = Path(os.path.abspath(__file__)).parent.parent
        root_path = src_path.parent
        cfg_path = os.path.join(root_path, 'setup.cfg')
        config.read(cfg_path)
        try:
            version = f"xbot2-gui-server {config['metadata']['version']}"
        except KeyError:
            version = str(get_distribution('xbot2-gui-server'))

        return web.Response(text=json.dumps(
            {
                'success': True,
                'message': 'got version',
                'version': version
            }
        ))


    @utils.handle_exceptions
    async def restart_program(self, req):
        """
        Restarts the current program, with file objects and descriptors
        cleanup
        """

        try:
            p = psutil.Process(os.getpid())
            for handler in p.open_files() + p.connections():
                os.close(handler.fd)
        except Exception as e:
            logging.error(e)

        python = sys.executable
        os.execl(python, python, *sys.argv)
