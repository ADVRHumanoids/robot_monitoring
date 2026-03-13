import asyncio
from aiohttp import web
import json, yaml
import re
import logging
import time
import os
from typing import List

import psutil
import threading

from .server import ServerBase
from . import utils

class ServerStatisticsHandler:

    def __init__(self, srv: ServerBase, config=dict()) -> None:
        
        # status broadcast rate (Hz)
        self.rate = 10.0
        
        # save server 
        self.srv = srv

        # 
        self.proc = psutil.Process(os.getpid())
        self.main_thread_id = threading.main_thread().native_id
        self.prev_cpu_time = self.main_thread_cpu_time()
        self.prev_wall_time = time.monotonic()

        # schedule tasks
        self.tasks = self.start_tasks()


    async def stop_tasks(self):
        for task in self.tasks:
            task.cancel()
        await asyncio.gather(*self.tasks, return_exceptions=True)


    def start_tasks(self):
       run_task = self.srv.schedule_task(self.run())
       return [run_task]

        
    def main_thread_cpu_time(self):
        for t in self.proc.threads():
            if t.id == self.main_thread_id:
                return t.user_time + t.system_time
        return None


    async def broadcast_stats(self):
        
        # compute CPU usage
        curr_cpu_time = self.main_thread_cpu_time()
        curr_wall_time = time.monotonic()
        cpu_usage = (curr_cpu_time - self.prev_cpu_time) / (curr_wall_time - self.prev_wall_time)
        self.prev_cpu_time = curr_cpu_time
        self.prev_wall_time = curr_wall_time

        # broadcast stats
        await self.srv.udp_send_to_all({
            'type': 'server_stats',
            'cpu_usage_main': cpu_usage * 100.0,
            'cpu_usage_total': self.proc.cpu_percent(),
            'memory_usage_MB': float(self.proc.memory_info().rss) / (1024 * 1024),
            'num_threads': self.proc.num_threads(),
        })
        

    async def run(self):
        await utils.sync_loop(self.broadcast_stats, dt=1.0/self.rate)()

