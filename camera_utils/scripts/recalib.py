import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import Int32
import tf
from tf.transformations import *
import tf2_ros
import tf2_geometry_msgs
import numpy as np

class marker_pose:
    def __init__(self, x, y):
        self.x = x
        self.y = y

marker_ID = -1
prev_marker_ID = -2
# marker_list = [ marker_pose(155, -5),
#                 marker_pose(445, 210),
#                 marker_pose(445, 590),
#                 marker_pose(445, 985),
#                 marker_pose(1600, 985),
#                 marker_pose(2230, 985),
#                 marker_pose(2230, 590),
#                 marker_pose(2230, 120),
#                 marker_pose(1600, 120),
#                 marker_pose(1600, 590)]
def callback(msg):
    global marker_ID
    global prev_marker_ID
    global pub
    global listener
    # global marker_list
    marker_ID = msg.data
    if marker_ID != -1:
        # if marker_ID != prev_marker_ID:
            print marker_ID, "detected"
            try:
                (trans1,rot1) = listener.lookupTransform('/marker_map_frame', '/dummy_base_link', rospy.Time(0))
                (trans2,rot2) = listener.lookupTransform('/map', '/markermap' + str(marker_ID), rospy.Time(0))

                transform_fixed = concatenate_matrices(translation_matrix([0,0,0]), quaternion_matrix([0, 0, -0.7071081, 0.7071055]))

                transform1 = concatenate_matrices(translation_matrix(trans1), quaternion_matrix(rot1))
                transform2 = concatenate_matrices(translation_matrix(trans2), quaternion_matrix(rot2))
                # global_trans_matrix = np.matmul(transform2, np.matmul(transform_fixed, transform1))
                global_trans_matrix = np.matmul(transform2, np.matmul(transform_fixed, transform1))
                global_trans_quaternion = quaternion_from_matrix(global_trans_matrix)
                global_trans_translation = translation_from_matrix(global_trans_matrix)

                init_pose_msg = PoseWithCovarianceStamped()
                init_pose_msg.header.stamp = rospy.Time.now()
                init_pose_msg.header.frame_id = "map"
                init_pose_msg.pose.pose.position.x = global_trans_translation[0]
                init_pose_msg.pose.pose.position.y = global_trans_translation[1]
                init_pose_msg.pose.pose.position.z = 0
                # print "x = ", init_pose_msg.pose.pose.position.x, " y = ", init_pose_msg.pose.pose.position.y
                init_pose_msg.pose.pose.orientation.x = global_trans_quaternion[0]
                init_pose_msg.pose.pose.orientation.y = global_trans_quaternion[1]
                init_pose_msg.pose.pose.orientation.z = global_trans_quaternion[2]
                init_pose_msg.pose.pose.orientation.w = global_trans_quaternion[3]
                pub.publish(init_pose_msg)
                # prev_marker_ID = marker_ID
            except Exception as e:
                print e


rospy.init_node("recalib", anonymous=True)
listener = tf.TransformListener()
pub = rospy.Publisher("initialpose", PoseWithCovarianceStamped, queue_size=1)
rospy.Subscriber("dewey_marker_ID", Int32, callback)
r = rospy.Rate(100.0)
while not rospy.is_shutdown():
    r.sleep()
