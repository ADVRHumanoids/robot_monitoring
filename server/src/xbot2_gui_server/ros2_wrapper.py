import asyncio
import types

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
import ament_index_python

from std_msgs.msg import String

class Ros2Utils:

    @staticmethod
    def master_alive():
        return True
     
    def __init__(self) -> None:
        
        # init 
        rclpy.init(signal_handler_options=rclpy.SignalHandlerOptions.NO)
        self.node = Node('xbot2_gui_server')
        self.ros_version = 2
        print('ROS2 initialized')

        # robot description subscriber if on ros2
        self.urdf = None

        def on_robot_desc_recv(msg):
            print('ros2: got urdf')
            self.urdf = msg.data
        
        self.urdf_sub = self.create_subscription(String, 
                                                'xbotcore/robot_description', 
                                                on_robot_desc_recv, 
                                                queue_size=1,
                                                latch=True)
        
    def get_urdf(self):
        return self.urdf


    def now(self):
        now = self.node.get_clock().now()
        def to_sec(self):
            return self.nanoseconds * 1e-9
        now.to_sec = types.MethodType(to_sec, now)
        return now


    def timestamp_to_sec(self, ts):
        return Time.from_msg(ts).nanoseconds * 1e-9


    async def spin_node(self, period_sec=0.016):
        while True:
            rclpy.spin_once(self.node, timeout_sec=0.005)
            await asyncio.sleep(period_sec - 0.005)


    def create_subscription(self, msg_class, name, callback, queue_size, latch=False, best_effort=False):
        
        if latch:
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
        else:
            durability=QoSDurabilityPolicy.BEST_AVAILABLE

        if best_effort:
            reliability=QoSReliabilityPolicy.BEST_EFFORT
        else:
            reliability=QoSReliabilityPolicy.RELIABLE

        qos_profile = QoSProfile(
            reliability=reliability,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=durability,
            depth=queue_size
        )
        
        return self.node.create_subscription(msg_class, name, callback, qos_profile=qos_profile)


    def create_publisher(self, msg_class, name, queue_size):
        return self.node.create_publisher(msg_class, name, queue_size)
    

    def create_client(self, srv_class, name):
        return self.node.create_client(srv_class, name)
    

    async def wait_for_server(self, client, timeout_sec=-1):
        tf = self.node.get_clock().now() + rclpy.duration.Duration(seconds=timeout_sec)
        while timeout_sec < 0 or self.node.get_clock().now() < tf:
            if client.wait_for_service(timeout_sec=0.005):
                return True
            await asyncio.sleep(0.1)
        raise TimeoutError(f'service {client.srv_name}: wait_for_server exceeded timeout')
    
    
    async def call(self, client, timeout_sec, **kwargs):
        req = client.srv_type.Request(**kwargs)
        future = client.call_async(req)
        tf = self.node.get_clock().now() + rclpy.duration.Duration(seconds=timeout_sec)
        while timeout_sec < 0 or self.node.get_clock().now() < tf:
            if future.done():
                return future.result()
            await asyncio.sleep(0.1)
        raise TimeoutError(f'service {client.srv_name}: call exceeded timeout')
   
    
    def get_package_share_directory(self, name):
       return ament_index_python.get_package_share_directory(name)
    
    
    def get_topic_names_and_types(self):
        return self.node.get_topic_names_and_types()
