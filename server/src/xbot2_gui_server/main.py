import os, sys
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.append(os.path.dirname(SCRIPT_DIR))

from .server import Xbot2WebServer
import yaml, json
import sys
import importlib
import rospy
import logging
from aiohttp import web

def main():

    print('starting server, initializing rospy..')

    # init rospy node
    rospy.init_node('xbot2_gui_server', disable_signals=True)

    logging.basicConfig(level=logging.INFO, force=True)
    
    # load config
    cfgpath = sys.argv[1]
    cfg = yaml.safe_load(open(cfgpath, 'r').read())

    # create server
    srv = Xbot2WebServer()
    srv.cfgpath = cfg

    # load default extensions
    extensions = []

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
    from .cartesian import CartesianHandler
    ext = CartesianHandler(srv, cfg.get('cartesian', {}))
    extensions.append(ext)

    # visual
    from .visual import VisualHandler
    ext = VisualHandler(srv, cfg.get('visual', {}))
    extensions.append(ext)

    # concert
    try:
        from .concert import ConcertHandler
        ext = ConcertHandler(srv, cfg.get('concert', {}))
        extensions.append(ext)
    except ModuleNotFoundError:
        pass

    # ecat
    try:
        from .ecat import EcatHandler
        ext = EcatHandler(srv, cfg.get('ecat', {}))
        extensions.append(ext)
    except ModuleNotFoundError:
        pass

    # horizon
    try:
        from .horizon import HorizonHandler
        ext = HorizonHandler(srv, cfg.get('horizon', {}))
        extensions.append(ext)
    except ModuleNotFoundError:
        pass

    # launcher
    try:
        from .launcher import Launcher
        ext = Launcher(srv, cfg.get('launcher', {}))
        extensions.append(ext)
    except ModuleNotFoundError:
        pass

    # parse requested pages
    requested_pages = cfg.get('requested_pages', [])
    for e in extensions:
        try:
            requested_pages += e.requested_pages
        except:
            pass 

    print(f'requested pages = {requested_pages}')

    async def requested_pages_handler(req):
        return web.Response(text=json.dumps({'requested_pages': requested_pages}))

    # get handler
    srv.add_route('GET', '/requested_pages', requested_pages_handler, 'requested_pages_handler')

    # run server
    srv.run_server()


if __name__ == '__main__':
    main()
