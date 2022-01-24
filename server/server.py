#!/usr/bin/env python3

import logging
import asyncio
from queue import Queue
from unittest import expectedFailure
import aiohttp
import argparse
from aiohttp import web
import json
import rospy
from xbot_msgs.msg import JointState

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
    js_msg_to_send['torRef'] = msg.effort_reference
    js_msg_to_send['tor'] = msg.effort
    js_msg_to_send['velRef'] = msg.velocity_reference
    js_msg_to_send['motVel'] = msg.motor_velocity
    js_msg_to_send['stamp'] = msg.header.stamp.to_sec()

js_sub = rospy.Subscriber('xbotcore/joint_states', JointState, on_js_recv, queue_size=1)

# recv first message to get joint names
js_msg : JointState = rospy.wait_for_message(js_sub.name, JointState)
print('first joint state received')
init_data = dict()
init_data['jnames'] = js_msg.name

# get init data handler
async def init_handler(request):
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
                print(f'unable to send message to clients: {type(e)} {e}')
                ws_to_remove.add(ws)
            except BaseException as e:
                print(f'error: {e}')
        
        for ws in ws_to_remove:
            CLIENTS.remove(ws)

        ws_to_remove.clear()

        i += 1
        await asyncio.sleep(0.1666)


# websocket handler
async def websocket_handler(request):

    print('New client connected!')

    ws = web.WebSocketResponse()
    await ws.prepare(request)

    CLIENTS.add(ws)

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

routes = [
    ('GET', '/ws', websocket_handler, 'websocket'),
    ('GET', '/init', init_handler, 'init'),
]

for route in routes:
    app.router.add_route(route[0], route[1], route[2], name=route[3])

app.router.add_static('/', args.root)

loop = asyncio.get_event_loop()
runner = web.AppRunner(app)

async def start_http_server(host=args.host, port=args.port):
    await runner.setup()
    site = web.TCPSite(runner, host, port)
    await site.start()

CLIENTS = set()


loop.create_task(js_broadcaster())
loop.run_until_complete(start_http_server())
loop.run_forever()
