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

string VOCAB_FILE;
string SETTINGS_FILE;

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

    mpSLAM->TrackMonocular(cv_ptr->image,cv_ptr->header.stamp.toSec());
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


    ros::Subscriber subscribe_images = nodeHandler.subscribe("/camera/image_raw", 1, &ImageGrabber::GrabImage,&igb);
    ros::Publisher publish_pointcloud = nodeHandler.advertise<sensor_msgs::PointCloud>("/pointcloud", 1);
    ros::Publisher publish_pose = nodeHandler.advertise<geometry_msgs::PoseStamped>("/pose", 1);

    while(ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();
    }

    SLAM.Shutdown();
    ros::shutdown();

    return 0;
}