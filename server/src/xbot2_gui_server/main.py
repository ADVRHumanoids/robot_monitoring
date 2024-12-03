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
import traceback as tb

def main():

    # cli
    parser = argparse.ArgumentParser(description='A modern UI for the Xbot2 framework, written in Qt6 / QML')
    parser.add_argument('config', type=str, nargs='?', help='path to config file')
    parser.add_argument('--port', '-p', type=int, default=8080, help='port for the UI server (it must be available on both TCP and UDP)')
    parser.add_argument('--launch-ui', '-u', action='store_true', help='run the UI frontend')
    args = parser.parse_args()

    # set verbose logging level
    logging.basicConfig(level=logging.DEBUG, force=True)
    
    # load config
    if args.config:
        cfgpath = args.config
        cfg = yaml.safe_load(open(cfgpath, 'r').read())
    else:
        cfgpath = __file__ 
        cfg = dict()

    # check ros master
    try:
        rospy.get_master().getPid()
    except Exception as e:
        print('no ros master running')
        exit(1)

    # init rospy node
    rospy.init_node('xbot2_gui_server', disable_signals=True)

    # create server
    srv = Xbot2WebServer()
    srv.cfgpath = cfgpath

    # load default extensions
    extensions = []

    # task that load all extensions after waiting for ros master
    def load_extensions():

        # wasm ui
        from .webui import WebUiHandler
        ext = WebUiHandler(srv, cfg.get('webui', {}))
        extensions.append(ext)

        # joint states
        try:
            print(f'Loading JointStateHandler')
            from .joint_states import JointStateHandler
            ext = JointStateHandler(srv, cfg.get('joint_states', {}))
            extensions.append(ext)
            print(ext)
        except BaseException as e:
            tb.print_exc()

        # joint device
        try:
            print(f'Loading JointDeviceHandler')
            from .joint_device import JointDeviceHandler
            ext = JointDeviceHandler(srv, cfg.get('joint_device', {}))
            extensions.append(ext)
            print(ext)
        except BaseException as e:
            tb.print_exc()

        # plugin
        try:
            print(f'Loading JointDeviceHandler')
            from .plugin import PluginHandler
            ext = PluginHandler(srv, cfg.get('plugin', {}))
            extensions.append(ext)
            print(ext)
        except BaseException as e:
            tb.print_exc()

        # theora video
        try:
            print(f'Loading TheoraVideoHandler')
            from .theora_video import TheoraVideoHandler
            ext = TheoraVideoHandler(srv, cfg.get('theora_video', {}))
            extensions.append(ext)
            print(ext)
        except BaseException as e:
            tb.print_exc()

        # launcher
        try:
            print(f'Loading Launcher')
            from .launcher import Launcher
            ext = Launcher(srv, cfg.get('launcher', {}))
            extensions.append(ext)
            print(ext)
        except:
            tb.print_exc()

        # cartesian
        try:
            print(f'Loading CartesianHandler')
            from .cartesian import CartesianHandler
            ext = CartesianHandler(srv, cfg.get('cartesian', {}))
            extensions.append(ext)
            print(ext)
        except:
            tb.print_exc()

        # speech
        # try:
        #     from .speech import SpeechHandler
        #     ext = SpeechHandler(srv, cfg.get('speech', {}))
        #     extensions.append(ext)
        #     print(ext)
        # except ModuleNotFoundError:
        #     pass
        # except BaseException as e:
        #     print('Exception ', type(e), e)  


        # visual
        try:
            print(f'Loading VisualHandler')
            from .visual import VisualHandler
            ext = VisualHandler(srv, cfg.get('visual', {}))
            extensions.append(ext)
            print(ext)
        except:
            tb.print_exc()

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

        # horizon
        if 'hhcm_calibration' in cfg.keys():
            try:  
                print(f'Loading VisualHandler')
                from .hhcm_calibration import HhcmCalibrationHandler
                ext = HhcmCalibrationHandler(srv, cfg.get('hhcm_calibration', {}))
                extensions.append(ext)
            except:
                tb.print_exc()

        
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

    # schedule extension loading task
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

