import os, sys
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.append(os.path.dirname(SCRIPT_DIR))

from .server import Xbot2WebServer
import yaml, json
import sys
import time
import importlib
import os
import logging
import argparse
from aiohttp import web
import asyncio

from . import ros_utils

def main():

    # cli
    parser = argparse.ArgumentParser(description='A modern UI for the Xbot2 framework, written in Qt6 / QML')
    parser.add_argument('config', type=str, nargs='?', help='path to config file')
    parser.add_argument('--port', '-p', type=int, default=8080, help='port for the UI server (it must be available on both TCP and UDP)')
    args = parser.parse_args()

    # set verbose logging level
    logging.basicConfig(level=logging.DEBUG, force=True)
    
    # load config
    if args.config:
        cfgpath = args.config
        cfg = yaml.safe_load(open(cfgpath, 'r').read())
    elif os.environ.get('XBOT2_GUI_CONFIG'):
        cfgpath = os.environ.get('XBOT2_GUI_CONFIG')
        cfg = yaml.safe_load(open(cfgpath, 'r').read())
    else:
        cfgpath = __file__ 
        cfg = dict()
        
    logging.info('config loaded from %s' % cfgpath)
        
    # wait for ros (1)
    while not ros_utils.RosWrapperClass.master_alive():
        logging.info('waiting for ros master')
        time.sleep(1)

    # create server
    srv = Xbot2WebServer()
    srv.cfgpath = cfgpath

   

    # load default extensions
    extensions = []

    # task that load all extensions after waiting for ros master
    def load_extensions():

        # load ros
        ros_utils.ros_handle = ros_utils.RosWrapperClass()

        # spin ros callbacks
        srv.schedule_task(ros_utils.ros_handle.spin_node())

        # wasm ui
        from .webui import WebUiHandler
        ext = WebUiHandler(srv, cfg.get('webui', {}))
        extensions.append(ext)

        # joint states
        from .joint_states import JointStateHandler
        ext = JointStateHandler(srv, cfg.get('joint_states', {}))
        extensions.append(ext)
        print('OK LOADED', ext)

        # joint device
        from .joint_device import JointDeviceHandler
        ext = JointDeviceHandler(srv, cfg.get('joint_device', {}))
        extensions.append(ext)
        print('OK LOADED', ext)

        # plugin
        from .plugin import PluginHandler
        ext = PluginHandler(srv, cfg.get('plugin', {}))
        extensions.append(ext)
        print('OK LOADED', ext)

        # theora video
        from .theora_video import TheoraVideoHandler
        ext = TheoraVideoHandler(srv, cfg.get('theora_video', {}))
        extensions.append(ext)
        print('OK LOADED', ext)

        # launcher
        try:
            from .launcher import Launcher
            ext = Launcher(srv, cfg.get('launcher', {}))
            extensions.append(ext)
        except ModuleNotFoundError:
            pass
        except BaseException as e:
            print('Exception ', type(e), e)  

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
            print(ext)
        except ModuleNotFoundError:
            pass
        except BaseException as e:
            print('Exception ', type(e), e)  


        # visual
        try:
            from .visual import VisualHandler
            ext = VisualHandler(srv, cfg.get('visual', {}))
            extensions.append(ext)
            print(ext)
        except BaseException as e:
            print('Exception ', type(e), e)  

        # concert
        if 'concert' in cfg.keys():
            try:
                from .concert import ConcertHandler
                ext = ConcertHandler(srv, cfg.get('concert', {}))
                extensions.append(ext)
                print(ext)    
            except BaseException as e:
                print('Exception ', type(e), e)  

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
        
        # dashboard
        try:
            from .dashboard import DashboardHandler
            ext = DashboardHandler(srv, cfg.get('dashboard', {}))
            extensions.append(ext)
            print(ext)
        except BaseException as e:
            print('Exception ', type(e), e)

        try:
            from .parameters import ParameterHandler
            ext = ParameterHandler(srv, cfg.get('parameters', {}))
            extensions.append(ext)
            print(ext)
        except BaseException as e:
            print('Exception ', type(e), e)

        srv.extensions = extensions

        print('load extensions completed', extensions)

    # load extensions
    load_extensions()

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

    # run server
    srv.run_server(port=args.port)


if __name__ == '__main__':
    main()

