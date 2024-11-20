#!/usr/bin/env python3

import sys
import time
import socket
import yaml
from dataclasses import asdict
import ipywidgets.widgets as widgets
from IPython.display import display

print(sys.executable)

from ecat_repl import ZmsgIO
from ecat_repl import FoeMaster
from ecat_repl import CtrlCmd
from ecat_repl import SdoCmd
from ecat_repl import SdoInfo

from ecat_repl import (
    master_cmd_stop,
    master_cmd_start,
    master_cmd_get_slave_descr,
    #
    flash_cmd_load_default,
    flash_cmd_save_flash,
    flash_cmd_load_flash,
    #
    ctrl_cmd_start,
    ctrl_cmd_stop,
    ctrl_cmd_fan,
    ctrl_cmd_led,
    ctrl_cmd_run_torque_calib,
    ctrl_cmd_set_home,
    ctrl_cmd_set_zero,
    ctrl_cmd_set_min_pos,
    ctrl_cmd_set_max_pos,
    ctrl_cmd_set_position,
    ctrl_cmd_set_velocity,
    ctrl_cmd_set_torque,
    ctrl_cmd_set_current,
    ctrl_cmd_dac_tune,
    ctrl_cmd_get_adc,
    ctrl_cmd_set_dac,
    ctrl_cmd_test_done,
    ctrl_cmd_test_error,
)

uri = "localhost:5555"
io = ZmsgIO(uri)
io.debug = False
scan_ids = []

io.doit(master_cmd_stop)

io.doit(master_cmd_start.set_args({'app_mode':'config_mode','use_ecat_pos_as_id':'true'}))

reply = io.doit(master_cmd_get_slave_descr)
yaml_msg = yaml.safe_load(reply['msg'])
scan_ids = yaml_msg.keys()
ids=list(scan_ids)
print(ids)

reply = io.doit(SdoInfo(u'SDO_NAME').set_bid(ids[0]))
yaml_msg = yaml.safe_load(reply['msg'])
sdo_names = yaml_msg
print(sdo_names)

reply = io.doit(SdoInfo(u'SDO_OBJD').set_bid(ids[0]))
yaml_msg = yaml.safe_load(reply['msg'])
sdo_infos = yaml_msg
print(sdo_infos)


for i in range(100):
    reply = io.doit(SdoCmd(rd_sdo=sdo_names,wr_sdo={}).set_bid(ids[0]))
    yaml_msg = yaml.safe_load(reply['msg'])
    print(yaml_msg)
    time.sleep(0.1)


#io.doit(SdoCmd(rd_sdo=['fw_ver'],wr_sdo={'board_id': 101}).set_bid(ids[0]))

#io.doit(flash_cmd_save_flash.set_bid(ids[0]))