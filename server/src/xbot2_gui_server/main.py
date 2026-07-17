import os, sys
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.append(os.path.dirname(SCRIPT_DIR))

from .server import Xbot2WebServer
import yaml, json
import time
import sys
import importlib
import logging
import argparse
from aiohttp import web
import asyncio

from . import ros_utils

def main():

    default_config_path = os.environ.get('XBOT2_GUI_SERVER_CONFIG', None)
    default_port = int(os.environ.get('XBOT2_GUI_SERVER_PORT', 8080))

    # cli
    parser = argparse.ArgumentParser(description='A modern UI for the Xbot2 framework, written in Qt6 / QML')
    parser.add_argument('config', type=str, default=default_config_path, help=f'path to config file (yaml); defaults to XBOT2_GUI_SERVER_CONFIG environment variable (default: {default_config_path})', nargs='?')
    parser.add_argument('--port', '-p', type=int, default=default_port, help='port for the UI server (it must be available on both TCP and UDP)')
    args = parser.parse_args()

    # set verbose logging level
    logging.basicConfig(level=logging.DEBUG, force=True)

    while not ros_utils.RosWrapper.master_alive():
        print('waiting for ros master')
        time.sleep(1.0)
    
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

        # load ros
        ros_utils.ros_handle = ros_utils.RosWrapper()

        # spin ros callbacks
        srv.schedule_task(ros_utils.ros_handle.spin_node())

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
            ('xbot2_gui_server.parameters', 'ParameterHandler'),
            ('xbot2_gui_server.diagnostics', 'DiagnosticsHandler'),
            ('xbot2_gui_server.server_stats', 'ServerStatisticsHandler')
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
