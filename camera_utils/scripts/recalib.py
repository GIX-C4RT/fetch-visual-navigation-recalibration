import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped

rospy.init_node()
pub = rospy.Publisher("initialpose", PoseWithCovarianceStamped)

while not rospy.is_shutdown():
    if ("dewey_camera", "marker_map_frame") transform exist:
        pose_recalib = PoseWithCovarianceStamped()
        pose_recalib.header.frame_id="map"
        pose_recalib.header.stamp = rospy.now()
        pose_recalib.pose.pose = get_baselink_pose_from_marker()

        pub.publish(pose_recalib)

