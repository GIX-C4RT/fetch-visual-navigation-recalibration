from utils import get_transform_matrix
import rospy
import numpy as np
import tf
from tf.transformations import *
import tf2_ros
import tf2_geometry_msgs

rospy.init_node('base_link_marker_map_bind', anonymous=True)
r = rospy.Rate(100.0)

# Publish a transform from base link to markermap
br = tf.TransformBroadcaster()
# Get T base to head
tf_buffer = tf2_ros.Buffer(rospy.Duration(300.0)) #tf buffer length
tf_listener = tf2_ros.TransformListener(tf_buffer)

while not rospy.is_shutdown():
    try:
        transform_HB = tf_buffer.lookup_transform("head_camera_rgb_optical_frame",
                                            "base_link", #source frame
                                            rospy.Time(0),
                                            rospy.Duration(5.0)) #get the tf at first available time

        # Get T head to marker by look up "marker_map_frame", "dewey_camera" I cannot directly publish head_camera_rgb fram to marker map because it will
        # causing jumps so this is only for our internal reference

        transform_MH = tf_buffer.lookup_transform("marker_map_frame",
                                            "dewey_camera", #source frame
                                            rospy.Time(0),
                                            rospy.Duration(5.0)) #get the tf at first available time

        m_HB = get_transform_matrix(transform_HB)
        m_MH = get_transform_matrix(transform_MH)
        m_MB = np.matmul(m_MH, m_HB)
        global_trans_quaternion = quaternion_from_matrix(m_MB)
        global_trans_translation = translation_from_matrix(m_MB)
        br.sendTransform(global_trans_translation,
                            global_trans_quaternion, 
                            rospy.Time.now() + rospy.Duration(1.0), 
                            "dummy_base_link", 
                            "marker_map_frame")
    except Exception as e:
        print e
    r.sleep()

# Publish the transform