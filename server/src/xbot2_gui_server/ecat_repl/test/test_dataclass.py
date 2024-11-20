from ecat_repl import MasterCmd, CtrlCmd, FoeMaster
from dataclasses import dataclass, field, asdict, InitVar


def test_dataclass():

    master_cmd_stop = MasterCmd(u'STOP_MASTER')
    master_cmd_start = MasterCmd(u'START_MASTER')
    master_cmd_get_slave_descr = MasterCmd(u'GET_SLAVES_DESCR')

    ctrl_cmd_start = CtrlCmd(u'CTRL_CMD_START')
    ctrl_cmd_stop = CtrlCmd(u'CTRL_CMD_STOP')

    cent_AC_m3 = FoeMaster(("../examples/fw_update/fw_test/cent_AC_m3.bin", 0xA550, "m3", 1))



