import rospy
import rospkg 
import asyncio
import utils

class Ros1Utils:
   
   @staticmethod
   def master_alive():
      try:
         rospy.get_master().getPid()
         return True
      except Exception as e:
         return False
     
   def __init__(self) -> None:
      # init rospy node
      rospy.init_node('xbot2_gui_server', disable_signals=True)
      self.ros_version = 1
      self.rospack = rospkg.RosPack()
      print('ROS1 initialized')

   def get_urdf(self):
      return rospy.get_param('xbotcore/robot_description', default=None)

   def now(self):
      return rospy.Time.now()

   def timestamp_to_sec(self, ts):
      return ts.to_sec()

   async def spin_node(self, period_sec=None):
      while True:
         await asyncio.sleep(1.0)

   def create_subscription(self, msg_class, name, callback, queue_size, latch=False, best_effort=False):
      return rospy.Subscriber(name, msg_class, callback, queue_size=queue_size)

   def create_publisher(self, msg_class, name, queue_size):
      return rospy.Publisher(name, msg_class, queue_size=queue_size)
    
   def create_client(self, srv_class, name):
      return rospy.ServiceProxy(name, srv_class)
   
   async def wait_for_server(self, client: rospy.ServiceProxy, timeout_sec=-1):
      tf = rospy.Time.now() + rospy.Duration(timeout_sec)
      while timeout_sec < 0 or rospy.Time.now() < tf:
         try:
            client.wait_for_service(timeout=0.005)
            return True
         except TimeoutError:
            await asyncio.sleep(0.1)
      raise TimeoutError(f'service {client.resolved_name}: wait_for_server exceeded timeout')
   
   async def call(self, client: rospy.ServiceProxy, timeout_sec, **kwargs):
      req = client.service_class._request_class(**kwargs)
      return await utils.to_thread(client, req)
   
   def get_package_share_directory(self, name):
      return self.rospack.get_path(name)
   
   def get_topic_names_and_types(self):
      return rospy.get_published_topics()