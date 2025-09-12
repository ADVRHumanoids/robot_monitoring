import os, sys
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.append(os.path.dirname(SCRIPT_DIR))

from .server import Xbot2WebServer
import yaml, json
import time
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
    args = parser.parse_args()

    # set verbose logging level
    logging.basicConfig(level=logging.INFO, force=True)

    while True:
        try:
            print('waiting for ros master')
            rospy.get_master().getPid()
            break
        except Exception as e:
            time.sleep(1.0)

    # init rospy node
    rospy.init_node('xbot2_gui_server', disable_signals=True)
    
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

        

        module_list = [
            'xbot2_gui_server.joint_states'
        ]

        # wasm ui
        from .webui import WebUiHandler
        ext = WebUiHandler(srv, cfg.get('webui', {}))
        extensions.append(ext)

        def load_extension(module_name, class_name, srv, config):
            print(f'>>> loading {module_name}.{class_name}')
            try:
                pkgname = module_name.split('.')[1]
                module = importlib.import_module(module_name)
                handler_class = getattr(module, class_name)
                ext = handler_class(srv, config.get(pkgname, {}))
                extensions.append(ext)
                print(f'<<< loaded {module_name}.{class_name}')
            except ModuleNotFoundError:
                print(f'Module {module_name} not found')
                import traceback
                traceback.print_exc()
            except AttributeError:
                print(f'Class {class_name} not found in module {module_name}')
                import traceback
                traceback.print_exc()
            except Exception as e:
                print(f'Error loading {module_name}.{class_name}: {type(e).__name__} - {e}')
                import traceback
                traceback.print_exc()

        # define extensions 
        ext_list = [
            ('xbot2_gui_server.joint_states', 'JointStateHandler'),
            ('xbot2_gui_server.joint_device', 'JointDeviceHandler'),
            ('xbot2_gui_server.plugin', 'PluginHandler'),
            ('xbot2_gui_server.theora_video', 'TheoraVideoHandler'),
            ('xbot2_gui_server.launcher', 'Launcher'),
            ('xbot2_gui_server.cartesian', 'CartesianHandler'),
            # ('xbot2_gui_server.speech', 'SpeechHandler'),
            ('xbot2_gui_server.visual', 'VisualHandler'),
            # ('xbot2_gui_server.concert', 'ConcertHandler'),
            ('xbot2_gui_server.ecat', 'EcatHandler'),
            ('xbot2_gui_server.horizon', 'HorizonHandler'),
            ('xbot2_gui_server.dashboard', 'DashboardHandler'),
            ('xbot2_gui_server.parameters', 'ParameterHandler')
        ]

        # load extensions
        for module_name, class_name in ext_list:
            load_extension(module_name, class_name, srv, cfg)

        print('loaded extensions:')
        for ext in extensions:
            print(' ', ext)

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

    # run server
    srv.run_server(port=args.port)


if __name__ == '__main__':
    main()

