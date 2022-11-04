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
from xbot_msgs.msg import JointState, Statistics2
from xbot_msgs.srv import GetPluginList, PluginStatus, PluginCommand
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import TwistStamped
from theora_image_transport.msg import Packet as TheoraPacket
from screen_session import Process
from std_srvs.srv import SetBool
from cartesian_interface.srv import SetControlMode, SetControlModeRequest
import base64


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
                cmd='xbot2-core',
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
        },
        'Config path': {
            'type': 'text',
            'default': '',
            'cmd': '--config {__value__}',
            'help': 'Select configuration file'
        }
    }
    srv.add_process(p)

    # tbd: processes from config
    p = Process(name='cartesio_manipulation', 
                cmd='roslaunch centauro_cartesio_config centauro_manipulation.launch',
                machine='arturo@localhost')

    srv.add_process(p)

    # tbd: processes from config
    p = Process(name='cartesio_move_base', 
                cmd='roslaunch centauro_cartesio centauro_car_model.launch',
                machine='arturo@localhost')

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

        # joint state subscriber
        self.pstat_sub = rospy.Subscriber('xbotcore/statistics', Statistics2, self.on_pstat_recv, queue_size=1)

        # compressed (i.e. jpeg) img sub
        self.img_sub = rospy.Subscriber('/usb_cam/image_raw/compressed', CompressedImage, self.on_img_recv, queue_size=1)

        # theora img sub
        self.img_sub = None

        # vel ref pub
        self.vref_pub = None

        # global joint state message
        self.js_msg_to_send = {} 

        # global process stats message
        self.proc_stat_msg_to_send = {}

        # global image message
        self.img_msg_to_send = {}

        # queue for th packets
        self.th_pkt_queue = asyncio.Queue(maxsize=0)
        self.th_hdr_pkt = []

        # web app
        self.app = web.Application()

        # event loop
        self.loop = asyncio.get_event_loop()

        logging.basicConfig(level=logging.DEBUG)

        # websocket clients
        self.ws_clients = set()

        # add routes
        routes = [
            ('GET', '/', self.root_handler, 'root'),
            ('GET', '/ws', self.websocket_handler, 'websocket'),
            ('GET', '/info', self.info_handler, 'init'),
            ('GET', '/get_video_stream', self.get_video_stream_handler, 'get_video_stream'),
            ('GET', '/cartesian/get_task_list', self.cartesian_get_task_list_handler, 'cartesian_get_task_list'),
            ('PUT', '/set_video_stream', self.set_video_stream_handler, 'set_video_stream'),
            ('PUT', '/proc', self.proc_handler, 'proc'),
            ('PUT', '/plugin/{plugin_name}/{command}', self.plugin_cmd_handler, 'plugin'),
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

        # shorthand        
        loop = self.loop

        # for each screen session, status, output broadcasters
        for p in self.procs.values():
            loop.create_task(self.proc_status_broadcaster(p))
            loop.create_task(self.proc_output_broadcaster(p, stream_type='stdout'))
            loop.create_task(self.proc_output_broadcaster(p, stream_type='stderr'))

        loop.create_task(self.js_broadcaster())
        loop.create_task(self.plugin_stat_broadcaster())
        loop.create_task(self.heartbeat())
        loop.create_task(self.video_broadcaster_jpeg())
        loop.create_task(self.video_broadcaster_theora())

        # first execute server start routine
        loop.run_until_complete(self.start_http_server(runner, host, port))

        # then, loop forever
        loop.run_forever()

    
    async def start_http_server(self, runner, host, port):
        await runner.setup()
        site = web.TCPSite(runner, host, port)
        await site.start()

    
    def on_pstat_recv(self, msg: Statistics2):
        self.proc_stat_msg_to_send['type'] = 'plugin_stats'
        
        for ts in msg.task_stats:
            th = next(filter(lambda x: x.name == ts.thread, msg.thread_stats))
            self.proc_stat_msg_to_send[ts.name] = {
                'run_time': ts.run_time,
                'expected_period': th.expected_period,
                'state': ts.state,
            }


    def on_img_recv(self, msg: CompressedImage):
        self.img_msg_to_send['type'] = 'jpeg'
        self.img_msg_to_send['data'] = base64.b64encode(msg.data).decode('ascii')

    
    def on_th_pkt_recv(self, msg: TheoraPacket):

        th_pkt = dict()
        th_pkt['type'] = 'theora'
        th_pkt['data'] = base64.b64encode(msg.data).decode('ascii')
        th_pkt['b_o_s'] = msg.b_o_s
        th_pkt['e_o_s'] = msg.e_o_s
        th_pkt['granulepos'] = msg.granulepos
        th_pkt['packetno'] = msg.packetno

        if msg.b_o_s == 1:
            self.th_hdr_pkt = [th_pkt]
            print(f'got header {len(self.th_hdr_pkt)}/3')
            return 

        if msg.granulepos == 0:
            self.th_hdr_pkt.append(th_pkt)
            print(f'got header {len(self.th_hdr_pkt)}/3')
            return

        _ = asyncio.run_coroutine_threadsafe(self.th_pkt_queue.put(th_pkt), self.loop)


    
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

        # plugins
        get_plugin_list = rospy.ServiceProxy('xbotcore/get_plugin_list', service_class=GetPluginList)
        plugin_list = get_plugin_list()
        init_data['plugins'] = plugin_list.plugins

        return web.Response(text=json.dumps(init_data))

    
    async def cartesian_get_task_list_handler(self, request):
        from cartesian_interface.srv import GetTaskList
        rospy.wait_for_service('cartesian/get_task_list', timeout=0.1)
        get_task_list = rospy.ServiceProxy('cartesian/get_task_list', GetTaskList)
        res = get_task_list()
        return web.Response(text=json.dumps(
            {
                'names': res.names,
                'types': res.types,
            }
        ))


    
    async def set_video_stream_handler(self, request):

        body = await request.text()
        body = json.loads(body)
        stream_name = body['stream_name']

        if self.img_sub is not None:
            self.img_sub.unregister()
        
        self.th_hdr_pkt.clear()

        self.img_msg_to_send = {}

        if stream_name == '':
            print('disconnected image subscriber')
            return web.Response(text=json.dumps({
                'success': True,
                'message': f'disconnected from image topic"',
                }))
        
        print(f'requested stream {stream_name}, registering subscriber and waiting for headers..')

        self.img_sub = rospy.Subscriber(stream_name, 
            TheoraPacket, self.on_th_pkt_recv, queue_size=20)

        # wait for headers
        iter = 0
        while len(self.th_hdr_pkt) < 3 and iter < 500:
            await asyncio.sleep(0.01)

        if len(self.th_hdr_pkt) < 3:
            return web.Response(text=json.dumps({
                'success': False,
                'message': f'failed to receive headers for "{stream_name}"',
                }))

        print('got headers')

        return web.Response(text=json.dumps({
                'success': True,
                'message': f'subscribed to "{stream_name}"',
                'hdr': self.th_hdr_pkt,
                }))

    
    async def get_video_stream_handler(self, request):

        topic_name_type_list = rospy.get_published_topics()
        vs_topics = list()
        for tname, ttype in topic_name_type_list:
            if ttype == 'theora_image_transport/Packet':
                vs_topics.append(tname)
        
        return web.Response(text=json.dumps({
            'success': True,
            'topics': vs_topics,
            }))


    async def plugin_cmd_handler(self, request):
        plugin_name = request.match_info.get('plugin_name', None)
        print(plugin_name)
        command = request.match_info.get('command', None)
        print(command)
        if command not in ('start', 'stop'):
            raise ValueError(f'invalid command {command}')
        switch = rospy.ServiceProxy(f'xbotcore/{plugin_name}/switch', service_class=SetBool)
        switch(command == 'start')
        return web.Response(text=json.dumps({'success': True}))

    
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
    
    
    # video broadcaster coroutine
    async def video_broadcaster_jpeg(self):

        while True:

            # periodic loop at 60 Hz
            await asyncio.sleep(0.0333)

            if not self.img_msg_to_send: 
                continue
            
            # serialize msg to json
            msg_str = json.dumps(self.img_msg_to_send)

            # iterate over sockets (one per client)
            for ws in self.ws_clients:

                try:
                    await ws.send_str(msg_str)  
                except ConnectionResetError as e:
                    pass
                except BaseException as e:
                    print(f'error: {e}')
            
    
    # video broadcaster coroutine
    async def video_broadcaster_theora(self):

        while True:

            # periodic loop at 60 Hz
            th_pkt = await self.th_pkt_queue.get()

            # print(f'b_o_s={th_pkt["b_o_s"]} e_o_s={th_pkt["e_o_s"]} packetno={th_pkt["packetno"]} granulepos={th_pkt["granulepos"]}')
            
            # serialize msg to json
            msg_str = json.dumps(th_pkt)

            # iterate over sockets (one per client)
            for ws in self.ws_clients:

                try:
                    await ws.send_str(msg_str)  
                except ConnectionResetError as e:
                    pass
                except BaseException as e:
                    print(f'error: {e}')
    
    
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

    
    # joint state broadcaster coroutine
    async def plugin_stat_broadcaster(self):

        while True:
            
            # periodic loop at 5 Hz
            await asyncio.sleep(0.20)

            if not self.proc_stat_msg_to_send:
                continue

            # serialize msg to json
            msg_str = json.dumps(self.proc_stat_msg_to_send)

            # iterate over sockets (one per client)
            for ws in self.ws_clients:
                
                # nothing to send, break
                if not self.js_msg_to_send:
                    break

                try:
                    await ws.send_str(msg_str)  
                except ConnectionResetError as e:
                    pass
                except BaseException as e:
                    print(f'error: {e}')
            
            


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
            await p.start(options=body['options'])
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
                msg = json.loads(msg.data)
                if msg['type'] == 'velocity_command':
                    await self.handle_velocity_command(msg)
                else:
                    print(f'Received unknown message from client: {msg}')
            elif msg.type == aiohttp.WSMsgType.ERROR:
                print('ws connection closed with exception %s' %
                    ws.exception())

        return ws

    
    async def handle_velocity_command(self, msg):
        task_name = msg['task_name']
        topic_name = rospy.resolve_name(f'cartesian/{task_name}/velocity_reference')
        ctrl_mode_srv_name = rospy.resolve_name(f'cartesian/{task_name}/set_control_mode')

        if self.vref_pub is None or self.vref_pub.resolved_name != topic_name:
            try:
                print(f'unregistering {self.vref_pub.name}')
            except:
                pass
            print(f'waiting {ctrl_mode_srv_name}')
            #rospy.wait_for_service(ctrl_mode_srv_name, timeout=1)
            ctrl_mode_srv = rospy.ServiceProxy(ctrl_mode_srv_name, service_class=SetControlMode)
            req = SetControlModeRequest()
            req.ctrl_mode = 'velocity'
            res = ctrl_mode_srv(req)
            print(f'returned {res}')
            print(f'advertising {topic_name}')
            self.vref_pub = rospy.Publisher(topic_name, TwistStamped, queue_size=1)

            

        rosmsg = TwistStamped()
        rosmsg.header.stamp = rospy.Time.now()
        rosmsg.header.frame_id = task_name
        vref = msg['vref']
        rosmsg.twist.linear.x = vref[0]
        rosmsg.twist.linear.y = vref[1]
        rosmsg.twist.linear.z = vref[2]
        rosmsg.twist.angular.x = vref[3]
        rosmsg.twist.angular.y = vref[4]
        rosmsg.twist.angular.z = vref[5]

        self.vref_pub.publish(rosmsg)
        


if __name__ == '__main__':
    main()
