import asyncio
from aiohttp import web

from os.path import join

from .server import ServerBase
from . import utils


class WebUiHandler:

    def __init__(self, srv: ServerBase, config=dict()) -> None:

        # save server object, register our handlers
        self.srv = srv

        self.srv.add_route('GET', '/webui/{file}',
                           self.webui_serve_file,
                           'webui_serve_file')

    
    @utils.handle_exceptions
    async def webui_serve_file(self, request):
        
        file = request.match_info['file']
        path = join('/home/alaurenzi/code/next_ui/build-robot_monitoring-WebAssembly_Qt_6_5_0_multi_threaded-Release/client', file)
        response = web.FileResponse(path)
        response.headers['Cross-Origin-Opener-Policy'] = 'same-origin'
        response.headers['Cross-Origin-Embedder-Policy'] = 'require-corp'
        return response

