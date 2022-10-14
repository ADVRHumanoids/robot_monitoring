#!/usr/bin/python3

import logging
import asyncio
import re
import contextlib
from queue import Queue
import aiohttp
import argparse
from aiohttp import web
import json
import rospy
from urdf_parser_py import urdf as urdf_parser
from xbot_msgs.msg import JointState
from xbot_msgs.srv import GetPluginList, PluginStatus, PluginCommand
from screen_session import Process


def main():
    
    # parse cmd line args
    parser = argparse.ArgumentParser()
    parser.add_argument('root', nargs='?', default='.')
    parser.add_argument('--host', default="0.0.0.0")
    parser.add_argument('--port', default=8080)
    parser.add_argument('--config', default=None, help='path to configuration file')
    args = parser.parse_args()

    # create and run server
    srv = Xbot2WebServer()

    # tbd: processes from config
    p = Process(name='xbot2', 
                cmd='xbot2-core --hw dummy --verbose',
                machine='arturo@localhost')

    p.cmdline = {
        'Verbose': {
            'type': 'check',
            'default': True,
            'cmd': '--verbose',
            'help': 'verbose console output'
        },
        'HW Type': {
            'type': 'combo',
            'options': ['auto', 'ec_imp', 'ec_pos', 'dummy', 'sim'],
            'default': 0,
            'cmd': ['', '--hw ec_imp', '--hw ec_pos', '--hw dummy', '--hw sim'],
            'help': 'Select hardware type'
        }
    }
    srv.add_process(p)

    # run forever
    srv.run_server(static=args.root, host=args.host, port=args.port)


class Xbot2WebServer:

    def __init__(self) -> None:
        
        # init ros node
        rospy.init_node('xbot2_web_server')

        # managed screen sessions
        self.procs = {}

        # joint state subscriber
        self.js_sub = rospy.Subscriber('xbotcore/joint_states', JointState, self.on_js_recv, queue_size=1)

        # global joint state message
        self.js_msg_to_send = {} 

        # web app
        self.app = web.Application()

        logging.basicConfig(level=logging.DEBUG)

        # websocket clients
        self.ws_clients = set()

        # add routes
        routes = [
            ('GET', '/', self.root_handler, 'root'),
            ('GET', '/ws', self.websocket_handler, 'websocket'),
            ('GET', '/info', self.info_handler, 'init'),
            ('PUT', '/proc', self.proc_handler, 'proc'),
        ]

        for route in routes:
            self.app.router.add_route(method=route[0], path=route[1], handler=route[2], name=route[3])

    
    def add_process(self, p: Process):
        self.procs[p.name] = p

    
    def run_server(self, static='.', host='0.0.0.0', port=8080):
        
        # serve static files
        self.app.router.add_static('/', static)

        # run
        runner = web.AppRunner(self.app)
        
        # schedule tasks
        loop = asyncio.get_event_loop()

        # for each screen session, status, output broadcasters
        for p in self.procs.values():
            loop.create_task(self.proc_status_broadcaster(p))
            loop.create_task(self.proc_output_broadcaster(p, stream_type='stdout'))
            loop.create_task(self.proc_output_broadcaster(p, stream_type='stderr'))

        loop.create_task(self.js_broadcaster())
        loop.create_task(self.heartbeat())

        # first execute server start routine
        loop.run_until_complete(self.start_http_server(runner, host, port))

        # then, loop forever
        loop.run_forever()

    
    async def start_http_server(self, runner, host, port):
        await runner.setup()
        site = web.TCPSite(runner, host, port)
        await site.start()


    def on_js_recv(self, msg):
        self.js_msg_to_send['type'] = 'joint_states'
        self.js_msg_to_send['name'] = msg.name
        self.js_msg_to_send['posRef'] = msg.position_reference
        self.js_msg_to_send['motPos'] = msg.motor_position
        self.js_msg_to_send['linkPos'] = msg.link_position
        self.js_msg_to_send['torRef'] = msg.effort_reference
        self.js_msg_to_send['tor'] = msg.effort
        self.js_msg_to_send['velRef'] = msg.velocity_reference
        self.js_msg_to_send['motVel'] = msg.motor_velocity
        self.js_msg_to_send['linkVel'] = msg.link_velocity
        self.js_msg_to_send['stamp'] = msg.header.stamp.to_sec()

    # get init data handler
    async def info_handler(self, request):

        init_data = dict()
        
        # screen session data
        proc_data = list()
        for p in self.procs.values():
            proc_data.append ({
                'name': p.name,
                'status': await p.status(),
                'cmdline': p.cmdline
            })

        init_data['proc_data'] = proc_data

        # joint state data
        try:
            js_msg : JointState = rospy.wait_for_message(self.js_sub.name, JointState, rospy.Duration(0.1))
            self.on_js_recv(js_msg)
            init_data['message'] = 'ok'
            init_data['success'] = True
        except rospy.ROSException as e:
            init_data['message'] = 'unable to receive joint state'
            init_data['success'] = False
            return web.Response(text=json.dumps(init_data))

        print('first joint state received')
        init_data['jstate'] = self.js_msg_to_send
        init_data['jnames'] = js_msg.name

        urdf = rospy.get_param('xbotcore/robot_description')
        model = urdf_parser.Robot.from_xml_string(urdf)
        print('got urdf')

        init_data['qmin'] = list()
        init_data['qmax'] = list()
        init_data['vmax'] = list()
        init_data['taumax'] = list()

        for jn in js_msg.name:
            joint = model.joint_map[jn]
            init_data['qmin'].append(joint.limit.lower)
            init_data['qmax'].append(joint.limit.upper)
            init_data['vmax'].append(joint.limit.velocity)
            init_data['taumax'].append(joint.limit.effort)

        return web.Response(text=json.dumps(init_data))

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
    
    
    # joint state broadcaster coroutine
    async def js_broadcaster(self):

        while True:

            # serialize msg to json
            js_str = json.dumps(self.js_msg_to_send)

            # iterate over sockets (one per client)
            for ws in self.ws_clients:
                
                # nothing to send, break
                if not self.js_msg_to_send:
                    break

                try:
                    await ws.send_str(js_str)  
                except ConnectionResetError as e:
                    pass
                except BaseException as e:
                    print(f'error: {e}')
            
            # periodic loop at 60 Hz
            await asyncio.sleep(0.01666)


    # root handler serves html for web app
    async def root_handler(self, request):
        return aiohttp.web.HTTPFound('/next_ui.html')

    # process status broadcaster
    async def proc_status_broadcaster(self, p: Process):

        while True:

            await asyncio.sleep(1.0)

            msg = {
                    'type': 'proc',
                    'content': 'status',
                    'name': p.name, 
                    'status': await p.status()
                }

            for ws in self.ws_clients:
                try:
                    msg_str = json.dumps(msg)
                    await ws.send_str(msg_str)  
                except ConnectionResetError as e:
                    pass
                except BaseException as e:
                    print(f'error: {e}')


    # broadcast process output
    async def proc_output_broadcaster(self, p: Process, stream_type: str):

        ansi_escape = re.compile(r'\x1B(?:[@-Z\\-_]|\[[0-?]*[ -/]*[@-~])')

        while True:

            if p.proc is None:
                await asyncio.sleep(0.1)
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

            for ws in self.ws_clients:
                try:
                    await ws.send_str(msg_str)  
                except ConnectionResetError as e:
                    pass
                except BaseException as e:
                    print(f'error: {e}')

    # process command handler
    async def proc_handler(self, request):
        body = await request.text()
        body = json.loads(body)
        p: Process = self.procs[body['name']]
        cmd = body['cmd']
        if cmd == 'start':
            await p.start()
        elif cmd == 'stop':
            await p.stop()
        return aiohttp.web.Response(text=json.dumps({'message': f'ok: {cmd} {p.name}', 'success': True}))


    # websocket handler
    async def websocket_handler(self, request):

        ws = web.WebSocketResponse()
        await ws.prepare(request)

        self.ws_clients.add(ws)
        print(f'new client connected (total is {len(self.ws_clients)})')

        async for msg in ws:
            if msg.type == aiohttp.WSMsgType.TEXT:
                if msg.data == 'close':
                    await ws.close()
                else:
                    print(f'Received message from client: {msg.data}')
                    await ws.send_str('Server thanks you for the kindness!')
            elif msg.type == aiohttp.WSMsgType.ERROR:
                print('ws connection closed with exception %s' %
                    ws.exception())

        return ws


if __name__ == '__main__':
    main()
