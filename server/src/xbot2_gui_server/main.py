import os, sys
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.append(os.path.dirname(SCRIPT_DIR))

from .server import Xbot2WebServer
import yaml
import sys
import importlib
import rospy
import logging

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

    # process
    from .process import ProcessHandler
    for extname, extcfg in cfg.items():
        if extcfg['type'] == 'process_handler':
            print(f'found process handler {extname}')
            ext = ProcessHandler(srv, extcfg)
            extensions.append(ext)
    
    # visual
    from .visual import VisualHandler
    ext = VisualHandler(srv, cfg.get('visual', {}))
    extensions.append(ext)

    # concert
    from .concert import ConcertHandler
    ext = ConcertHandler(srv, cfg.get('concert', {}))
    extensions.append(ext)

    # run server
    srv.run_server()


if __name__ == '__main__':
    main()
