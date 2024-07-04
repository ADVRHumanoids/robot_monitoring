import os, sys
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.append(os.path.dirname(SCRIPT_DIR))

from .server import Xbot2WebServer
import yaml, json
import sys
import importlib
import rospy
import logging
import argparse
from aiohttp import web
import asyncio

def main():

    # cli
    parser = argparse.ArgumentParser(description='A modern UI for the Xbot2 framework, written in Qt6 / QML')
    parser.add_argument('config', type=str, nargs='?', help='path to config file')
    parser.add_argument('--port', '-p', type=int, default=8080, help='port for the UI server (it must be available on both TCP and UDP)')
    parser.add_argument('--launch-ui', '-u', action='store_true', help='run the UI frontend')
    args = parser.parse_args()

    # set verbose logging level
    logging.basicConfig(level=logging.INFO, force=True)
    
    # load config
    if args.config:
        cfgpath = args.config
        cfg = yaml.safe_load(open(cfgpath, 'r').read())
    else:
        cfgpath = __file__ 
        cfg = dict()

    # create server
    srv = Xbot2WebServer()
    srv.cfgpath = cfgpath

    # load default extensions
    extensions = []

    # task that load all extensions after waiting for ros master
    async def load_extensions():

        while True:
            try:
                await srv.log('waiting for ros master')
                rospy.get_master().getPid()
                break
            except Exception as e:
                await asyncio.sleep(1.0)

        await srv.log('ros master is alive')

        # init rospy node
        rospy.init_node('xbot2_gui_server', disable_signals=True)

        # wasm ui
        from .webui import WebUiHandler
        ext = WebUiHandler(srv, cfg.get('webui', {}))
        extensions.append(ext)

        # joint states
        from .joint_states import JointStateHandler
        ext = JointStateHandler(srv, cfg.get('joint_states', {}))
        extensions.append(ext)

        # joint device
        from .joint_device import JointDeviceHandler
        ext = JointDeviceHandler(srv, cfg.get('joint_device', {}))
        extensions.append(ext)

        # plugin
        from .plugin import PluginHandler
        ext = PluginHandler(srv, cfg.get('plugin', {}))
        extensions.append(ext)

        # theora video
        from .theora_video import TheoraVideoHandler
        ext = TheoraVideoHandler(srv, cfg.get('theora_video', {}))
        extensions.append(ext)

        # cartesian
        try:
            from .cartesian import CartesianHandler
            ext = CartesianHandler(srv, cfg.get('cartesian', {}))
            extensions.append(ext)
        except ModuleNotFoundError:
            pass

        # speech
        try:
            from .speech import SpeechHandler
            ext = SpeechHandler(srv, cfg.get('speech', {}))
            extensions.append(ext)
        except ModuleNotFoundError:
            pass

        # visual
        from .visual import VisualHandler
        ext = VisualHandler(srv, cfg.get('visual', {}))
        extensions.append(ext)

        # concert
        if 'concert' in cfg.keys():
            from .concert import ConcertHandler
            print('concert')
            ext = ConcertHandler(srv, cfg.get('concert', {}))
            print('concert')
            extensions.append(ext)

        # ecat
        if 'ecat' in cfg.keys():
            from .ecat import EcatHandler
            ext = EcatHandler(srv, cfg.get('ecat', {}))
            extensions.append(ext)

        # horizon
        if 'horizon' in cfg.keys():
            from .horizon import HorizonHandler
            ext = HorizonHandler(srv, cfg.get('horizon', {}))
            extensions.append(ext)

        # launcher
        try:
            from .launcher import Launcher
            ext = Launcher(srv, cfg.get('launcher', {}))
            extensions.append(ext)
        except ModuleNotFoundError:
            pass
        
        # dashboard
        try:
            from .dashboard import DashboardHandler
            ext = DashboardHandler(srv, cfg.get('dashboard', {}))
            extensions.append(ext)
        except BaseException as e:
            print('Exception ', type(e), e)

        srv.extensions = extensions

        print('load extensions completed', extensions)

    # schedule extension loading task
    srv.schedule_task(load_extensions())

    async def requested_pages_handler(req):
        # parse requested pages
        requested_pages = cfg.get('requested_pages', [])
        for e in extensions:
            try:
                requested_pages += e.requested_pages
            except:
                pass 
        print(requested_pages)
        return web.Response(text=json.dumps({'requested_pages': requested_pages}))

    srv.add_route('GET', '/requested_pages', requested_pages_handler, 'requested_pages_handler')

    # run ui client if required
    async def run_ui():

        proc = await asyncio.create_subprocess_shell(
            f'bash -ic "new-xbot2-gui -p {args.port}"',
                    stdout=asyncio.subprocess.PIPE,
                    stderr=asyncio.subprocess.STDOUT,
                    stdin=asyncio.subprocess.PIPE)
        
        while True:
            try:
                l = await proc.stdout.readline()
                if len(l) == 0:
                    retcode = await proc.wait()
                    print(f'[ui] process exited with {retcode}')
                    sys.exit(retcode)
                l = l.decode()
                print('[ui]', l, end='')
            except KeyboardInterrupt:
                return

    if args.launch_ui:
        srv.schedule_task(run_ui())

    # run server
    srv.run_server(port=args.port)


if __name__ == '__main__':
    main()

