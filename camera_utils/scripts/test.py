import rospy
import time
from geometry_msgs.msg import PoseStamped
rospy.init_node("test", anonymous=True)
r = rospy.Rate(100.0)
pub = rospy.Publisher("move_base_simple/goal", PoseStamped, queue_size=1)
time.sleep(2)
msg = PoseStamped()
msg.header.stamp = rospy.Time.now()
msg.header.frame_id = "map"
msg.pose.position.x = 16
msg.pose.position.y = 5.9
msg.pose.position.z = 0
msg.pose.orientation.w = 1
msg.pose.orientation.x = 0
msg.pose.orientation.y = 0
msg.pose.orientation.z = 0

pub.publish(msg)