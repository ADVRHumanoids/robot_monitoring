import rospy 
from std_srvs.srv import Trigger
import yaml
import sys

rospy.init_node('stupid_speech_handler_node')

srvs = yaml.safe_load(open(sys.argv[1], 'r'))['speech']['commands'].values()

for s in srvs:
    def cb(req):
        print(s)
        return False, 'you are in the wrong state, please be more careful in the future!'
    print(f'advertised {s}')
    rospy.Service(s, Trigger, cb)

rospy.spin()
