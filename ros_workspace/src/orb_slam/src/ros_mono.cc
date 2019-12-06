#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <unistd.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>

#include "../../../include/System.h"
#include"../../../include/MapPoint.h"

#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point32.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

using namespace std;
using namespace tf;

// define required constants
string VOCAB_FILE;
string SETTINGS_FILE;
ros::Publisher pose_pub;
ros::Publisher pointcloud_pub;
ros::Subscriber image_sub;

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM2::System* pSLAM):mpSLAM(pSLAM){}

    void GrabImage(const sensor_msgs::ImageConstPtr& msg);
    void GrabPose(ros::Publisher publish_pose);
    void GrabPointCloud(ros::Publisher publish_pointcloud);

    ORB_SLAM2::System* mpSLAM;
};

void ImageGrabber::GrabImage(const sensor_msgs::ImageConstPtr& msg)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvShare(msg);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv::Mat pose = mpSLAM->TrackMonocular(cv_ptr->image,cv_ptr->header.stamp.toSec());

    if(pose.empty()) return;

    tf::Matrix3x3 rh_cameraPose(-pose.at<float>(0,0),  pose.at<float>(0,1),  pose.at<float>(0,2),
                                -pose.at<float>(1,0),  pose.at<float>(1,1),  pose.at<float>(1,2),
                                 pose.at<float>(2,0), -pose.at<float>(2,1), -pose.at<float>(2,2));
    tf::Vector3 rh_cameraTranslation(pose.at<float>(0,3), pose.at<float>(1,3), -pose.at<float>(2,3) );
    tf::Matrix3x3 rotation270degZX(0, 0, 1, -1, 0, 0, 0,-1, 0);
    tf::Quaternion q;
    rh_cameraPose.getRotation(q);
    geometry_msgs::PoseStamped p;
    p.header.frame_id = "world";
    p.pose.position.x = rh_cameraTranslation[0];
    p.pose.position.y = rh_cameraTranslation[1];
    p.pose.position.z = rh_cameraTranslation[2];
    p.pose.orientation.x = q[0];
    p.pose.orientation.y = q[1];
    p.pose.orientation.z = q[2];
    p.pose.orientation.w = q[3];
    pose_pub.publish(p);

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Mono");
    ros::start(); ros::Rate loop_rate(30);
    ros::NodeHandle nodeHandler;

    nodeHandler.param<std::string>("VOCAB_FILE", VOCAB_FILE, "vocab.txt");
    nodeHandler.param<std::string>("SETTINGS_FILE", SETTINGS_FILE, "settings.yaml");

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(VOCAB_FILE,SETTINGS_FILE,ORB_SLAM2::System::MONOCULAR,true);

    ImageGrabber igb(&SLAM);
    pointcloud_pub = nodeHandler.advertise<sensor_msgs::PointCloud>("/pointcloud", 1);
    pose_pub = nodeHandler.advertise<geometry_msgs::PoseStamped>("/pose", 1);
    image_sub = nodeHandler.subscribe("/camera/image_raw", 1, &ImageGrabber::GrabImage, &igb);

    while(ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();
    }

    SLAM.Shutdown();
    ros::shutdown();

    return 0;
}
