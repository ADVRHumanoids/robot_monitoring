#!/usr/bin/env python3

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

# parse cmd line args
parser = argparse.ArgumentParser()
parser.add_argument('root', nargs='?', default='.')
parser.add_argument('--host', default="localhost")
parser.add_argument('--port', default=8080)
args = parser.parse_args()


# rospy subscribers
rospy.init_node('xbot2_gui_server')
js_msg_to_send = dict()
def on_js_recv(msg: JointState):
    global js_msg_to_send
    js_msg_to_send['type'] = 'joint_states'
    js_msg_to_send['name'] = msg.name
    js_msg_to_send['posRef'] = msg.position_reference
    js_msg_to_send['motPos'] = msg.motor_position
    js_msg_to_send['linkPos'] = msg.link_position
    js_msg_to_send['torRef'] = msg.effort_reference
    js_msg_to_send['tor'] = msg.effort
    js_msg_to_send['velRef'] = msg.velocity_reference
    js_msg_to_send['motVel'] = msg.motor_velocity
    js_msg_to_send['linkVel'] = msg.link_velocity
    js_msg_to_send['stamp'] = msg.header.stamp.to_sec()

js_sub = rospy.Subscriber('xbotcore/joint_states', JointState, on_js_recv, queue_size=1)

# processes
class Process:

    def __init__(self):
        self.name = None
        self.cmdList = []
        self.proc: asyncio.subprocess.Process = None
        self.cmd = ''
        self.hostname = 'arturo@localhost'

    async def attach(self):

        if self.proc is not None:
            return True
        
        # first check if session running, attach if it is
        cmd = f'screen -xr {self.name}'
        args = ['-tt', self.hostname, cmd]
        print('executing command ssh ' + ' '.join(args))
        tmp_proc = await asyncio.create_subprocess_exec('/usr/bin/ssh', *args,
                stdout=asyncio.subprocess.PIPE,
                stderr=asyncio.subprocess.PIPE,
                stdin=asyncio.subprocess.PIPE)

        # if this command fails immediately, no session exists 
        try:
            # give it 1 sec
            retcode = await asyncio.wait_for(tmp_proc.wait(), 1.0)
            print(f'cmd returned {retcode}: session does not exist')
            return False

        except asyncio.TimeoutError:
            # timeout expired, we got the session
            print('got session')

            # run lifetime handler
            self.proc = tmp_proc
            loop.create_task(self._lifetime_handler())

            return True

    async def start(self):
        
        # first check if session running, attach if it is
        await self.attach()

        if self.proc is not None:
            return True

        # create new session
        cmd = f'rm -rf /tmp/{self.name}_log; screen -mS {self.name} -L -Logfile /tmp/{self.name}_log {self.cmd};'
        args = ['-tt', self.hostname, cmd]
        print('executing command ssh ' + ' '.join(args))
        self.proc = await asyncio.create_subprocess_exec('/usr/bin/ssh', *args,
                stdout=asyncio.subprocess.PIPE,
                stderr=asyncio.subprocess.PIPE,
                stdin=asyncio.subprocess.PIPE)
        print('DONE executing command ssh ' + ' '.join(args))

        # run lifetime handler
        loop.create_task(self._lifetime_handler())

        return True
    
    async def stop(self):
        self.proc.stdin.write('\x03\n'.encode())
        await self.proc.stdin.drain()  

    async def is_running(self):
        with contextlib.suppress(asyncio.TimeoutError):
            await asyncio.wait_for(self.proc.wait(), 1e-6)
        return self.proc.returncode is None

    async def status(self):

        await self.attach()

        if self.proc is None:
            return 'Stopped'

        if await self.is_running():
            return 'Running'
        else:
            return 'Stopped'

    async def _lifetime_handler(self):
        
        if self.proc is None:
            return

        retcode = await self.proc.wait()

        print(f'process {self.name} died with exit code {retcode}')

        self.proc = None


procs = dict()

p = Process()
p.name = 'Xbot2'
p.cmd = 'xbot2-core --hw dummy --verbose'
p.cmdList = ['Homing', 'ROS Ctrl']
procs[p.name] = p

p = Process()
p.name = 'Gazebo'
p.cmd = 'mon launch centauro_gazebo centauro_world.launch'
p.cmdList = ['Homing', 'ROS Ctrl']
procs[p.name] = p

p = Process()
p.name = 'EcatMaster'
p.cmd = 'ecat_master'
p.cmdList = ['Brakes OFF', 'Fan ON']
procs[p.name] = p


# get init data handler
async def info_handler(request):

    init_data = dict()
    init_data['message'] = 'ok'
    init_data['success'] = True

    # process status
    proc_data = list()
    for p in procs.values():
        proc_data.append ({
            'name': p.name,
            'status': await p.status(),
            'cmdList': p.cmdList
        })

    init_data['proc_data'] = proc_data

    
    try:
        js_msg : JointState = rospy.wait_for_message(js_sub.name, JointState, rospy.Duration(1))
        on_js_recv(js_msg)
    except rospy.ROSException as e:
        init_data['message'] = 'unable to receive joint state'
        init_data['success'] = False
        return web.Response(text=json.dumps(init_data))

    print('first joint state received')
    init_data['jstate'] = js_msg_to_send
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

# joint state broadcaster coroutine
async def js_broadcaster():

    i = 0
    while True:
        ws_to_remove = set()
        for ws in CLIENTS:
            if not js_msg_to_send:
                break
            try:
                js_str = json.dumps(js_msg_to_send)
                await ws.send_str(js_str)  
            except ConnectionResetError as e:
                print(f'unable to send message to client: ({e}); will remove client.')
                ws_to_remove.add(ws)
            except BaseException as e:
                print(f'error: {e}')
        
        for ws in ws_to_remove:
            CLIENTS.remove(ws)
            print(f'removing client ({len(CLIENTS)} left)')

        ws_to_remove.clear()

        i += 1
        await asyncio.sleep(0.01666)


# root
async def root_handler(request):
    return aiohttp.web.HTTPFound('/next_ui.html')

# process status broadcaster
async def proc_status_broadcaster(p: Process):

    while True:

        await asyncio.sleep(1.0)

        msg = {
                'type': 'proc',
                'content': 'status',
                'name': p.name, 
                'status': await p.status()
            }

        for ws in CLIENTS:
            try:
                msg_str = json.dumps(msg)
                await ws.send_str(msg_str)  
                print(msg_str)
            except ConnectionResetError as e:
                pass
            except BaseException as e:
                print(f'error: {e}')



async def proc_output_broadcaster(p: Process):

    ansi_escape = re.compile(r'\x1B(?:[@-Z\\-_]|\[[0-?]*[ -/]*[@-~])')
    
    while True:

        if p.proc is None:
            await asyncio.sleep(0.1)
            continue

        line = await p.proc.stdout.readline()
        line = line.decode().strip()
        line = ansi_escape.sub('', line)

        msg = {
            'type': 'proc',
            'content': 'output',
            'name': p.name,
            'data': line
        }

        for ws in CLIENTS:
            try:
                msg_str = json.dumps(msg)
                await ws.send_str(msg_str)  
            except ConnectionResetError as e:
                pass
            except BaseException as e:
                print(f'error: {e}')

# process command handler
async def proc_handler(request):
    body = await request.text()
    body = json.loads(body)
    p: Process = procs[body['name']]
    cmd = body['cmd']
    if cmd == 'start':
        await p.start()
    elif cmd == 'stop':
        await p.stop()
    return aiohttp.web.Response(text=json.dumps({'message': f'ok: {cmd} {p.name}', 'success': True}))


# websocket handler
async def websocket_handler(request):


    ws = web.WebSocketResponse()
    await ws.prepare(request)

    CLIENTS.add(ws)
    print(f'new client connected (total is {len(CLIENTS)})')

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

    print('websocket connection closed')

    return ws


# run web app
app = web.Application()

logging.basicConfig(level=logging.DEBUG)

# add routes
routes = [
    ('GET', '/', root_handler, 'root'),
    ('GET', '/ws', websocket_handler, 'websocket'),
    ('GET', '/info', info_handler, 'init'),
    ('PUT', '/proc', proc_handler, 'proc'),
]

for route in routes:
    app.router.add_route(route[0], route[1], route[2], name=route[3])

# serve static files
app.router.add_static('/', args.root)

# run
loop = asyncio.get_event_loop()
runner = web.AppRunner(app)

async def start_http_server(host=args.host, port=args.port):
    await runner.setup()
    site = web.TCPSite(runner, host, port)
    await site.start()

CLIENTS = set()

for p in procs.values():
    loop.create_task(proc_status_broadcaster(p))
    loop.create_task(proc_output_broadcaster(p))
loop.create_task(js_broadcaster())
loop.run_until_complete(start_http_server())
loop.run_forever()
