#include <iostream>
#include <cmath>
#include <string>
#include <aruco/aruco.h>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Int32.h>
#include <cv_bridge/cv_bridge.h>
#include <tf/transform_broadcaster.h>

// Current marker map that is seened by the robot
int marker_ID = -1;
// Aruco config and detect objects
static aruco::CameraParameters camera;
static aruco::MarkerDetector Detector;
static aruco::MarkerMap mmap_array[10];

// call back function for each frame
void image_callback(const sensor_msgs::Image::ConstPtr& msg)
{
    static tf::TransformBroadcaster br;     // tf boardcaster
    static aruco::MarkerMapPoseTracker MMTracker;   // marker map tracker
    // Marker map's internel marker id is incremental this is a lookup table
    static int marker_lookuptable[60] = {0, 0, 0, 0, 0, 0, 
                                        1, 1, 1, 1, 1, 1, 
                                        2, 2, 2, 2, 2, 2, 
                                        3, 3, 3, 3, 3, 3, 
                                        4, 4, 4, 4, 4, 4, 
                                        5, 5, 5, 5, 5, 5, 
                                        6, 6, 6, 6, 6, 6, 
                                        7, 7, 7, 7, 7, 7, 
                                        8, 8, 8, 8, 8, 8, 
                                        9, 9, 9, 9, 9, 9};
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        exit(-1);
    }
    
    // detect the markers
    auto markers=Detector.detect(cv_ptr->image);//0.05 is the marker size
    if(markers.size() != 0)
    {
        // figure out which marker map is it and load the tracker parameter
        marker_ID = marker_lookuptable[markers[0].id];
        MMTracker.setParams(camera, mmap_array[marker_ID]);
        MMTracker.estimatePose(markers);
        cv::Mat tvec;
        cv::Mat rvec;
        if (MMTracker.isValid())
        {
            tvec = MMTracker.getTvec();
            rvec = MMTracker.getRvec();
            std::cout << "rvec: " << rvec << "; tvec: " << tvec << std::endl;
        }
        // getvec returns a 1 by 3 opencv mat
        if (rvec.size().width == 3)
        {
            tf::Transform transform;

            // set transform translation
            transform.setOrigin(tf::Vector3(tvec.at<float>(0,0), tvec.at<float>(0,1), tvec.at<float>(0,2)));
            
            // get rotation angle
            float theta = cv::norm(rvec);
            std::cout << "theta: " << theta << std::endl;

            // get rotation axis
            rvec /= theta;
            tf::Vector3 axis(rvec.at<float>(0,0), rvec.at<float>(0,1), rvec.at<float>(0,2));
            std::cout << "axis: " << rvec << std::endl;

            tf::Quaternion q(axis, theta);
            transform.setRotation(q);

            br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "dewey_camera", "marker_map_frame"));
        }
    }
    else
    {
        marker_ID = -1;
    }
}

int main(int argc,char **argv)
{
    // Setting up ARUCO
    static const std::string config_prefix = "/home/fetch/camera_utils/src/camera_utils/config/markermap/Configuration_meter";
    static const std::string config_postfix = ".yml";
    std::string config_path;
    for(int i = 0; i < 10; i++)
    {
        config_path = config_prefix + std::to_string(i) + config_postfix;
        std::cout << "mmap " << std::to_string(i) << " reading from: " << config_path << std::endl;
        mmap_array[i].readFromFile(config_path);
    }
    camera.readFromXMLFile("/home/fetch/camera_utils/src/camera_utils/config/fetch_camera_calib.yml");
    Detector.setDictionary("ARUCO_MIP_36h12");
    
    // Setting up ROS
    ros::init(argc, argv, "local_camera_pose_estimator");
    ros::NodeHandle n;
    ros::Publisher marker_ID_pub = n.advertise<std_msgs::Int32>("dewey_marker_ID", 1);
    ros::Subscriber sub = n.subscribe("head_camera/rgb/image_raw", 1, image_callback);
    ros::Rate loop_rate(50);
    std_msgs::Int32 msg;
    while(ros::ok())
    {
        msg.data = marker_ID;
        marker_ID_pub.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
