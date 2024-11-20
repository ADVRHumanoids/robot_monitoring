import sys
import time
import socket
import yaml
import math
from  pprint import pprint as pp
from dataclasses import asdict
#import ipywidgets.widgets as widgets
#from IPython.display import display

# from .zmsg_io import ZmsgIO

print("Python path :", sys.executable)
print("Python version :", sys.version)

from .zmsg_io import ZmsgIO
from .base_cmd import SdoCmd

status_cmds = {
    'FAN_ON':           0x0026,
    'FAN_OFF':			0x0062,
    'LED_ON':           0x0019,
    'LED_OFF':          0x0091,
    'RELEASE_BRAKE':    0x00BD,
    'ENGAGE_BRAKE':     0x00DB,
    'LOAD_DFLT_CM':     0x00C1,
    'LOAD_CM':          0x00C4,
    'POWER_MOD_ON':     0x00A5,
    'POWER_MOD_OFF':    0x005A,
    'SET_DIRECT_MODE':  0x004F,

}
reply_cmds = {
    'CMD_DONE':         0x7800,
    'CMD_WORKING':		0xD000,
    'CMD_ERROR':		0xAA00,
    'CMD_NOCOND':       0xBB00,
    'CMD_INVALID':      0xEE00,
}

# set default uri
uri = "localhost:5555"
_io = ZmsgIO(uri)

def set_uri(uri):
    global _io
    print('new uri {}'.format(uri))
    _io = ZmsgIO(uri)
    return _io

def reply_cmd(cmd):
    reply = _io.doit(cmd)
    yaml_msg = yaml.safe_load(reply['msg'])
    return yaml_msg

def sdo_filter(snames:list, sdos:dict):
    return {key:sdos[key] for key in snames}

def read_sdo(rd_sdos:list, ids:list):
    d = dict()
    for iD in ids:
        yaml_msg = reply_cmd(SdoCmd(rd_sdo=rd_sdos,wr_sdo={}).set_bid(iD))
        d[iD] = yaml_msg
    return d

def write_sdo(wr_sdo:dict, ids:list):
    d = dict()
    wr_keys = list(wr_sdo.keys())
    for iD in ids:
        yaml_msg = reply_cmd(SdoCmd(rd_sdo=wr_keys,wr_sdo=wr_sdo).set_bid(iD))
        d[iD] = yaml_msg
    return d

def ctrl_status_cmd(ctrl_cmd:int, bid:int):
    wr_sdo = {'ctrl_status_cmd': ctrl_cmd}
    rd_sdo = ['ctrl_status_cmd_ack']
    write_sdo(wr_sdo,[bid])
    msg = read_sdo(rd_sdo,[bid])[bid]
    print(hex(msg['ctrl_status_cmd_ack']))
    return msg['ctrl_status_cmd_ack'] == ctrl_cmd + reply_cmds['CMD_DONE']

def ctrl_status_cmd_str(ctrl_cmd:str, bid:int):
    return ctrl_status_cmd(status_cmds[ctrl_cmd], bid) 