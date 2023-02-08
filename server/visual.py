import asyncio
from aiohttp import web
import json

import rospy
from urdf_parser_py import urdf as urdf_parser

from .server import ServerBase
from . import utils


class VisualHandler:

    def __init__(self, srv: ServerBase, config=dict()) -> None:

        # config
        self.rate = config.get('rate', 10.0)

        # save server object, register our handlers
        self.srv = srv
        self.srv.add_route('GET', '/visual/get_mesh/{uri}',
                           self.visual_get_mesh_handler,
                           'visual_get_mesh_handler')
        self.srv.add_route('GET', '/visual/get_mesh_entities',
                           self.visual_get_mesh_entities,
                           'visual_get_mesh_entities')

        # vel ref pub
        self.vref_pub = None

    
    @utils.handle_exceptions
    async def visual_get_mesh_handler(self, request):
        
        uri = request.match_info.get('uri', None)
        path = utils.resolve_ros_uri(uri)
        print(uri, path)
        return web.FileResponse(path)

    
    @utils.handle_exceptions
    async def visual_get_mesh_entities(self, request):
        
        # parse urdf
        urdf = rospy.get_param('xbotcore/robot_description')
        model = urdf_parser.Robot.from_xml_string(urdf)

        # get list of visuals
        visuals = dict()
        for lname, l in model.link_map.items():
            for c in l.collisions:
                if isinstance(c.geometry, urdf_parser.Mesh):
                    visuals[lname] = c.geometry.filename
        
        return web.json_response(visuals)


