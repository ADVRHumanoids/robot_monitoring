import math

from .base_cmd import MasterCmd, CtrlCmd, FlashCmd

controller_type = {
    "00_idle_ctrl":        0x00,
    "3B_motor_pos_ctrl":   0x3B,
    "3C_link_pos_ctrl":    0x3C,
    "D4_impedance_ctrl":   0xD4,
    "71_motor_speed_ctrl": 0x71,
    "CC_current_ctrl":     0xCC,
}

master_cmd_stop = MasterCmd(u'STOP_MASTER')
master_cmd_start = MasterCmd(u'START_MASTER')
master_cmd_get_slave_descr = MasterCmd(u'GET_SLAVES_DESCR')

#
flash_cmd_save_flash = FlashCmd(u'SAVE_PARAMS_TO_FLASH')
flash_cmd_load_default = FlashCmd(u'LOAD_DEFAULT_PARAMS')
flash_cmd_load_flash = FlashCmd(u'LOAD_PARAMS_FROM_FLASH')

# Motor
def ctrl_cmd_start(): return CtrlCmd(u'CTRL_CMD_START')
ctrl_cmd_stop = CtrlCmd(u'CTRL_CMD_STOP')
ctrl_cmd_fan = CtrlCmd(u'CTRL_FAN')
ctrl_cmd_led = CtrlCmd(u'CTRL_LED')
ctrl_cmd_run_torque_calib = CtrlCmd('CTRL_RUN_TORQUE_CALIB')
ctrl_cmd_set_home = CtrlCmd(u'CTRL_SET_HOME')
ctrl_cmd_set_zero = CtrlCmd(u'CTRL_SET_ZERO_POSITION')
ctrl_cmd_set_min_pos = CtrlCmd(u'CTRL_SET_MIN_POSITION')
ctrl_cmd_set_max_pos = CtrlCmd(u'CTRL_SET_MAX_POSITION')
ctrl_cmd_set_position = CtrlCmd(u'CTRL_SET_POSITION')
ctrl_cmd_set_velocity = CtrlCmd(u'CTRL_SET_VELOCITY')
ctrl_cmd_set_torque = CtrlCmd(u'CTRL_SET_TORQUE')
ctrl_cmd_set_current = CtrlCmd(u'CTRL_SET_CURRENT')
ctrl_cmd_set_gains = CtrlCmd(u'CTRL_SET_GAINS')

# Ft6
ctrl_cmd_dac_tune = CtrlCmd(u'CTRL_DAC_TUNE')
ctrl_cmd_get_adc = CtrlCmd(u'CTRL_GET_ADC')
ctrl_cmd_set_dac = CtrlCmd(u'CTRL_SET_DAC')
ctrl_cmd_set_dac_flash = CtrlCmd(u'CTRL_SET_DAC_FLASH')

#
ctrl_cmd_test_error = CtrlCmd(u'CTRL_TEST_ERROR')
ctrl_cmd_test_done = CtrlCmd(u'CTRL_TEST_DONE')


def gen_cmds(cmds: list):

    for cmd in cmds:
        if 'board_id_list' in cmd:
            id_list = cmd['board_id_list']
            del cmd['board_id_list']
            for _id in id_list:
                for key in ['ctrl_cmd', 'flash_cmd', 'trajectory_cmd', 'slave_sdo_cmd']:
                    if key in cmd:
                        cmd[key]['board_id'] = _id
                        break
                    #else:
                        #print("no board_id key ...")
                yield cmd
        else:
            yield cmd


