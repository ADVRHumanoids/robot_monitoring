import rospkg
import os
import functools
from concurrent.futures import ThreadPoolExecutor 
import asyncio
from aiohttp import web 
import json 
import traceback

th_executor = ThreadPoolExecutor(max_workers=8)

rospack = rospkg.RosPack()


def str2bool(string):
    valid = string.lower() in ('true', 'false', 'yes', 'no', '1', '0')
    if not valid:
        raise ValueError(f'string "{string}" can not be converted to bool')
    return string.lower() in ('true', 'yes', '1')


def resolve_ros_uri(uri: str):
    
    if uri.startswith('package://'):
        tokens = uri[10:].split('/')
        pkg = tokens[0]
        pkg_path = rospack.get_path(pkg)
        return os.path.join(pkg_path, *tokens[1:])
    else:
        return uri


async def to_thread(func, *args, **kwargs):
    loop = asyncio.get_running_loop()
    func = functools.partial(func, *args, **kwargs)
    fut = loop.run_in_executor(th_executor, func)
    await asyncio.wait([fut])
    if fut.exception():
        raise fut.exception()
    return fut.result()


def handle_exceptions(func):

    @functools.wraps(func)
    async def async_wrapper(self, *args, **kwargs):
        try:
            return await func(self, *args, **kwargs)
        except BaseException as e:
            traceback.print_exc()
            return web.Response(text=json.dumps({
                    'success': False, 
                    'message': f'{e.__class__.__name__} {e}',
                    }))

    return async_wrapper

