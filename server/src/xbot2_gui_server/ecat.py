import asyncio
from aiohttp import web
import json
import math
import time

from xbot2_cli.ecat_context import Context as EcatContext
from xbot2_cli.ecat_context import Arguments as EcatArgs

from .server import ServerBase
from . import utils


class EcatHandler:

    def __init__(self, srv: ServerBase, config=dict()) -> None:


        # request ui page
        self.requested_pages = ['Ecat']

        # config
        self.uri = config.get('uri', 'localhost:5555')

        # save server object, register our handlers
        self.srv = srv

        # cache
        self.ctx = EcatContext(cli=False, uri=self.uri)


        self.srv.add_route('GET', '/ecat/get_slave_list',
                           self.get_slave_list,
                           'ecat_get_slave_list')

        self.srv.add_route('GET', '/ecat/get_sdo_list',
                           self.get_sdo_list,
                           'ecat_get_sdo_list')

        self.srv.add_route('GET', '/ecat/read_sdo',
                           self.read_sdo,
                           'ecat_read_sdo')

        self.srv.add_route('GET', '/ecat/get_cmd_list',
                           self.get_cmd_list,
                           'ecat_get_cmd_list')
        
        self.srv.add_route('POST', '/ecat/write_sdo',
                           self.write_sdo,
                           'ecat_write_sdo')


    @utils.handle_exceptions
    async def get_slave_list(self, req):

        ids = self.ctx.list_id(EcatArgs(), verbose=False)

        ids = [id for id in ids if id > 0]

        if len(ids) == 0:

            return web.Response(text=json.dumps(
                {
                    'success': False,
                    'message': 'failed to list ids'
                }
                ))

        else:

            return web.Response(text=json.dumps(
                {
                    'success': True,
                    'message': f'ok',
                    'id': ids
                }
                ))


    @utils.handle_exceptions
    async def get_cmd_list(self, req):

        return web.Response(text=json.dumps(
            {
                'success': True,
                'message': f'ok',
                'cmd': list(self.ctx.cmd_dict.keys())
            }
            ))


    @utils.handle_exceptions
    async def get_sdo_list(self, req: web.Request):

        # get id parameter from request
        ids = list(map(int, req.query['id'].split(',')))
        
        # refresh cache if needed
        if self.ctx.sdo_list is None or self.ctx.sdo_dict is None:
            print('cache is empty, refreshing')
            self.ctx.update_cache()

        for id in ids:
            if id not in self.ctx.sdo_dict.keys():
                print(f'esc id {id} not found in cache, refreshing...')
                await utils.to_thread(self.ctx.update_cache)
                print('...done')
                break

        # fill sdo list
        sdos = []
        for id in ids:
            sdo_id = self.ctx.sdo_dict[id]
            sdo_id.sort()
            for s in sdo_id:
                if s not in sdos:
                    sdos.append(s)


        if len(sdos) == 0:

            return web.Response(text=json.dumps(
                {
                    'success': False,
                    'message': 'failed to list sdos'
                }
                ))

        else:

            return web.Response(text=json.dumps(
                {
                    'success': True,
                    'message': f'ok',
                    'sdo': list(sdos)
                }
                ))


    @utils.handle_exceptions
    async def read_sdo(self, req: web.Request):

        # get id parameter from request
        ids = list(map(int, req.query['id'].split(',')))
        sdo = req.query['sdo']
        
        sdo = self.ctx.read_sdo(EcatArgs(id=ids, name=sdo))

        if len(sdo) == 0:

            return web.Response(text=json.dumps(
                {
                    'success': False,
                    'message': 'failed to list sdos'
                }
                ))

        else:

            return web.Response(text=json.dumps(
                {
                    'success': True,
                    'message': f'ok',
                    'sdo': sdo
                }
                ))


    @utils.handle_exceptions
    async def write_sdo(self, req: web.Request):

        # get id parameter from request
        ids = list(map(int, req.query['id'].split(',')))
        
        # handle cmd
        cmd = req.query.get('cmd', None)
        if cmd is not None:
            self.ctx.exec_cmd(EcatArgs(id=ids, cmd=cmd))
            return web.Response(text=json.dumps(
                {
                    'success': True,
                    'message': f'successfully executed command {cmd} for ids {ids}',
                }
                ))

        # handle write sdo
        sdo = req.query['sdo']
        value = req.query['value']

        self.ctx.write_sdo(EcatArgs(id=ids, name=sdo, value=value))

        return web.Response(text=json.dumps(
                        {
                            'success': True,
                            'message': f'ok',
                        }
                        ))