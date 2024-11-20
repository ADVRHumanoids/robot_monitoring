#!/usr/bin/env python3
import os
import yaml
import argparse
from subprocess import PIPE, Popen

from ecat_repl import gen_cmds
from ecat_repl import repl_cmd
from ecat_repl import ZmsgIO
from ecat_repl import PipeIO


            
def test_repl(yaml_file):

    d = yaml.load(open(yaml_file, 'r'), Loader=yaml.FullLoader)

    if d['start_srv'] :
        proc = Popen(["repl"], stdout=PIPE, stderr=PIPE)

    if d['use_zmq'] :
        io = ZmsgIO(d['uri'])
    else:
        io = PipeIO()

    #print(d)

    cnt = 1 #d["cmd_cnt"]
    while cnt:

        print("cmd_exec_cnt", cnt)
        cnt -= 1

        test_dict = {"type": "ECAT_MASTER_CMD", "ecat_master_cmd": {"type": "GET_SLAVES_DESCR"}}
        io.doit(test_dict)
        
        if not 'cmds' in d:
            continue

        for cmd_dict in gen_cmds(d['cmds']):
            io.doit(cmd_dict)
            #time.sleep(0.01)

        #time.sleep(0.05)

    if d['start_srv'] :
        proc.terminate()
        print(proc.stdout.readlines())

    print("Exit")

