import rospkg
import os
import functools
from concurrent.futures import ThreadPoolExecutor 
import asyncio
from aiohttp import web 
import json 

th_executor = ThreadPoolExecutor(max_workers=8)

rospack = rospkg.RosPack()


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
    async def async_wrapper(*args, **kwargs):
        try:
            return await func(*args, **kwargs)
        except BaseException as e:
            print(f'[{func.__name__}] exception occurred: {e}')
            return web.Response(text=json.dumps({
                    'success': False, 
                    'message': str(e),
                    }))

    return async_wrapper

