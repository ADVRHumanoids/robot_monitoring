import yaml
import json
import zmq
from protobuf_to_dict import protobuf_to_dict, dict_to_protobuf
from dataclasses import asdict, is_dataclass

from .proto import repl_cmd_pb2 as repl_cmd
from .base_io import gen_cmds
from .base_cmd import MasterCmd, CtrlCmd, FlashCmd

class MultiPartMessage(object):
    header = None

    @classmethod
    def recv(cls, socket):
        """Reads key-value message from socket, returns new instance."""
        return cls.from_msg(socket.recv_multipart())

    @property
    def msg(self):
        return [self.header]

    def send(self, socket, identity=None):
        """Send message to socket"""
        msg = self.msg
        if identity:
            msg = [identity] + msg
        return socket.send_multipart(msg, flags=zmq.NOBLOCK)


class HelloMessage(MultiPartMessage):
    header = b"HELLO"


class EscCmdMessage(MultiPartMessage):
    header = b"ESC_CMD"

    def __init__(self, cmd):
        self.cmd = cmd

    @property
    def msg(self):
        # returns list of all message frames as a byte-string:
        return [self.header, self.cmd]


class EcatMasterCmdMessage(MultiPartMessage):
    header = b"MASTER_CMD"

    def __init__(self, cmd):
        self.cmd = cmd

    @property
    def msg(self):
        # returns list of all message frames as a byte-string:
        return [self.header, self.cmd]


class ZmsgIO(object):

    REQUEST_TIMEOUT = 1000

    def __init__(self, uri):
        #  Prepare our context and sockets
        self.ctx = zmq.Context()
        self.uri = uri
        self.debug = False

    def _try_request(self, uri: str, cmd: dict):
        """ Model One: Simple Retry and Failover """
        client = self.ctx.socket(zmq.REQ)
        client.setsockopt(zmq.LINGER, 0)  # Terminate early
        client.connect("tcp://" + uri)
        # ##############################################################
        # prepare msg to send
        # dict -> protobuf -> serialize to string -> send through socket
        cmd_pb = dict_to_protobuf(repl_cmd.Repl_cmd, cmd)
        if self.debug:
            print(cmd_pb)
        if cmd['type'] in ["ECAT_MASTER_CMD", "FOE_MASTER"]:
            cmd_msg = EcatMasterCmdMessage(cmd_pb.SerializeToString())
        else:
            cmd_msg = EscCmdMessage(cmd_pb.SerializeToString())
        cmd_msg.send(client)
        # ##############################################################
        #
        poll = zmq.Poller()
        poll.register(client, zmq.POLLIN)
        socks = dict(poll.poll(ZmsgIO.REQUEST_TIMEOUT))
        if socks.get(client) == zmq.POLLIN:
            rep_data = client.recv()
            rep = repl_cmd.Cmd_reply()
            # fill protobuf mesg
            rep.ParseFromString(rep_data)
            # print(rep)
            d = protobuf_to_dict(rep)
            yaml_msg = yaml.safe_load(d['msg'])
            json_msg = json.dumps(yaml_msg)
            # print(json_msg)
            if d['type'] == 'NACK':
                print(rep)
        else:
            d = {}
        poll.unregister(client)
        client.close()
        return d

    def doit(self, cmd):
        """ send cmd """
        _cmd = asdict(cmd) if is_dataclass(cmd) else cmd
        if self.debug:
            print(_cmd)
        return self._try_request(self.uri, _cmd)

    def doit4ids(self, ids, cmd):
        """ for each id send cmd """
        _cmd = asdict(cmd) if is_dataclass(cmd) else cmd
        _cmd['board_id_list'] = ids
        for c in gen_cmds([_cmd]):
            yield self._try_request(self.uri, _cmd)
