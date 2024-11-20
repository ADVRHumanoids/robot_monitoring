import os
from protobuf_to_dict import protobuf_to_dict, dict_to_protobuf

from ecat_repl.proto import repl_cmd_pb2 as repl_cmd

class PipeIO(object):

    nrt_pipe_path = '/proc/xenomai/registry/rtipc/xddp/'
    rt_pipe_path = '/tmp/nrt_pipes/'

    def __init__(self):

        self.wr_to, self.rd_from = self._open_pipes()

    def _open_pipes(self):
        for dir_path in [self.rt_pipe_path, self.nrt_pipe_path]:
            if os.path.isdir(dir_path):
                try:
                    _wr_to = open(os.path.join(dir_path, 'repl_in'), 'wb')
                    _rd_from = open(os.path.join(dir_path, 'repl_out'), 'rb')
                    return _wr_to, _rd_from
                except IOError as e:
                    print(e)
                    raise e

    def send_to(self, cmd: dict):
        cmd_pb = dict_to_protobuf(repl_cmd.Repl_cmd, cmd)
        serialized = cmd_pb.SerializeToString()
        msg_size = len(serialized)
        self.wr_to.write(msg_size.to_bytes(4, byteorder='little'))
        self.wr_to.write(serialized)
        self.wr_to.flush()

    def recv_from(self):
        rep_size = int.from_bytes(self.rd_from.read(4), byteorder='little')
        rep = repl_cmd.Cmd_reply()
        rep.ParseFromString(self.rd_from.read(rep_size))
        return rep


