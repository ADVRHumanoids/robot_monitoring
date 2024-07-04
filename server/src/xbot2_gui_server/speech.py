import asyncio
from aiohttp import web
import json
import threading

import queue
import time
import base64

import vosk

import rospy
from std_srvs.srv import Trigger

from .server import ServerBase
from . import utils


class SpeechHandler:

    def __init__(self, srv: ServerBase, config=dict()) -> None:


        # save server object, register our handlers
        self.srv = srv
        self.srv.register_ws_coroutine(self.handle_ws_msg)
        self.srv.schedule_task(self.run())
        self.rate = 30

        self.srv.add_route('POST', '/speech/enable_commands',
                           self.enable_commands_handler,
                           'speech_enable_commands')


        self.srv.add_route('POST', '/speech/send_command',
                           self.send_command_handler,
                           'speech_send_command')

        self.srv.add_route('GET', '/speech/info',
                           self.info_handler,
                           'speech_info')
    

        self.audio_queue = queue.Queue()
        self.text_queue = queue.Queue()

        self.vosk_thread = threading.Thread(target=self.vosk_thread_main)
        self.vosk_thread.setDaemon(daemonic=True)
        self.vosk_thread.start()

        self.grammar = config.get('grammar', [])
        self.cmd_dict = config.get('commands', {})
        self.prompt = config['prompt']
        self.cmd_queue = None

        self.enable_commands = False

        # audio (this is for playing received audio - testing only)
        # audio = pyaudio.PyAudio()
        # FORMAT = pyaudio.paInt16
        # CHANNELS = 1
        # RATE = 16000
        # self.audio_playing = False
        # self.audio_stream = audio.open(format=FORMAT,
        #                 channels=CHANNELS,
        #                 rate=RATE,
        #                 output=True,
        #                 frames_per_buffer=1024,
        #                 stream_callback=self.audio_callback,
        #                 start=True)
        

    # def audio_callback(self, in_data, frame_count,
    #      time_info,
    #      status_flags):
        
    #     buf_lwm = 2
    #     buf_hwm = 4  # this causes delay!
    #     buf_size = self.audio_queue.qsize()
        
    #     if not self.audio_playing and buf_size < 6:
    #         print(f'buffering ({buf_size}/{buf_hwm})')
    #         return '\0'*frame_count*2, pyaudio.paContinue
        
    #     self.audio_playing = True
        
    #     if buf_size < buf_lwm:
    #         self.audio_playing = False
    #         return '\0'*frame_count*2, pyaudio.paContinue

    #     return self.audio_queue.get(), pyaudio.paContinue


    @utils.handle_exceptions
    async def send_command_handler(self, req):

        cmd : str = req.rel_url.query['cmd']
        cmd = cmd.lower()

        success, msg = await self.send_command(cmd)

        return web.Response(text=json.dumps(
            {
                'success': success,
                'message': msg
            }
            ))


    @utils.handle_exceptions
    async def enable_commands_handler(self, req):

        self.enable_commands = utils.str2bool(req.rel_url.query['active'])

        return web.Response(text=json.dumps(
            {
                'success': True,
                'message': 'voice commands active:' + str(self.enable_commands)
            }
            ))


    @utils.handle_exceptions
    async def info_handler(self, req):

        return web.Response(text=json.dumps(
            {
                'body': {
                    'active': self.enable_commands,
                    'prompt': self.prompt,
                    'commands': list(self.cmd_dict.keys())
                },
                'success': True,
                'message': 'voice commands active:' + str(self.enable_commands)
            }
            ))


    def vosk_thread_main(self):

        md = vosk.Model(lang='en-us')

        grammar = list(self.cmd_dict.keys()) + [self.prompt] + ['[unk]']

        rec = vosk.KaldiRecognizer(md, 16000, json.dumps(grammar))

        while True:

            data = self.audio_queue.get()

            res = ''
            partial = False
            
            if rec.AcceptWaveform(data):
                res = rec.Result()
            else:
                res = rec.PartialResult()
                partial = True

            # print(res)
            if not partial:
                res = json.loads(res)['text']
                if len(res) > 0:
                    print(f'"{res}"')
                    self.text_queue.put(res)


    async def send_command(self, cmd):
        if cmd not in self.cmd_dict.keys():
            await self.srv.ws_send_to_all(dict(type='speech_cmd', cmd='__invalid__'))
            return False, 'invalid command'
        else:
            await self.srv.ws_send_to_all(dict(type='speech_cmd', cmd=cmd))
            srv = self.cmd_dict[cmd]
            srv = rospy.ServiceProxy(srv, Trigger)
            try:
                res = srv()
                if res.success:
                    await self.srv.ws_send_to_all(dict(type='speech_cmd', cmd='__done__'))
                    return True, 'ok'
                else:
                    await self.srv.ws_send_to_all(dict(type='speech_cmd', cmd='__error__', msg=res.message))
                    return False, res.message
            except:
                await self.srv.ws_send_to_all(dict(type='speech_cmd', cmd='__error__', msg='service failure'))
                return False, 'service failure'


    async def listen_to_command(self, timeout):
        print('started listening')
        await self.srv.ws_send_to_all(dict(type='speech_cmd', cmd='__start__'))
        try:
            cmd = await asyncio.wait_for(self.cmd_queue.get(), timeout=timeout)
            cmd = cmd.replace('[unk]', '').strip()
            print(f'processing command "{cmd}"')
            await self.send_command(cmd)
                
        except asyncio.TimeoutError as e:
            await self.srv.ws_send_to_all(dict(type='speech_cmd', cmd='__timeout__'))
            print('command timeout')
        finally:
            print('command end')
            self.cmd_queue = None 
            # await self.srv.ws_send_to_all(dict(type='speech_cmd', cmd='__end__'))
    

    async def run(self):

        while True:
            
            try:
                while True:
                    
                    txt = self.text_queue.get_nowait().strip()
                    await self.srv.ws_send_to_all(dict(type='speech_text', text=txt))

                    if self.enable_commands and txt == self.prompt and self.cmd_queue is None:
                        self.cmd_queue = asyncio.Queue()
                        self.srv.schedule_task(self.listen_to_command(timeout=8))
                        
                    elif self.cmd_queue is not None:
                        await self.cmd_queue.put(txt)

            except queue.Empty as e:
                pass

            await asyncio.sleep(1./self.rate)


    async def handle_ws_msg(self, msg, proto, ws):
        if msg['type'] == 'speech':
            frame = base64.b64decode(msg['data'])
            self.audio_queue.put(frame)

