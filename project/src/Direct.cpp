//
// Created by song on 18/02/06.
//

#include "../include/ORB_SLAM2_Stereo.h"


#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>

#include "geometry_msgs/TransformStamped.h"
#include "../include/System.h"
#include <tf/transform_broadcaster.h>

using namespace std;







class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM2::System *pSLAM) : mpSLAM(pSLAM) {}

    void GrabStereo(const sensor_msgs::ImageConstPtr &msgLeft, const sensor_msgs::ImageConstPtr &msgRight);

    ORB_SLAM2::System *mpSLAM;
    bool do_rectify;
    cv::Mat M1l, M2l, M1r, M2r;
};


bool yamlReading(string filename,Mat K_l, Mat& K_r,Mat&  P_l,Mat& P_r,Mat& R_l,Mat& R_r,Mat& D_l,Mat& D_r)
{
    cv::FileStorage fsSettings(filename, cv::FileStorage::READ);


    if (!fsSettings.isOpened())
    {
        cerr << "ERROR: Wrong path to settings" << endl;
        return -1;
    }



    fsSettings["LEFT.K"] >> K_l;
    fsSettings["RIGHT.K"] >> K_r;

    fsSettings["LEFT.P"] >> P_l;
    fsSettings["RIGHT.P"] >> P_r;

    fsSettings["LEFT.R"] >> R_l;
    fsSettings["RIGHT.R"] >> R_r;

    fsSettings["LEFT.D"] >> D_l;
    fsSettings["RIGHT.D"] >> D_r;

    int rows_l = fsSettings["LEFT.height"];
    int cols_l = fsSettings["LEFT.width"];
    int rows_r = fsSettings["RIGHT.height"];
    int cols_r = fsSettings["RIGHT.width"];

    if (K_l.empty() || K_r.empty() || P_l.empty() || P_r.empty() || R_l.empty() || R_r.empty() || D_l.empty() || D_r.empty() ||
        rows_l == 0 || rows_r == 0 || cols_l == 0 || cols_r == 0)
    {
        cerr << "ERROR: Calibration parameters to rectify stereo are missing!" << endl;
        return false;
    }
    return true;
}



int main(int argc, char **argv)
{
    ros::init(argc, argv, "Stereo");
    ros::start();



    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[0]);
    //类初始化
    ImageGrabber igb(&SLAM);
    



    //读取配置

    // Load settings related to stereo calibration

    cv::Mat K_l, K_r, P_l, P_r, R_l, R_r, D_l, D_r;
    if(yamlReading(argv[0],  K_l, K_r, P_l, P_r, R_l, R_r, D_l, D_r))
        return -1;



//    cv::initUndistortRectifyMap(K_l, D_l, R_l, P_l.rowRange(0, 3).colRange(0, 3), cv::Size(cols_l, rows_l), CV_32F, igb.M1l, igb.M2l);
//    cv::initUndistortRectifyMap(K_r, D_r, R_r, P_r.rowRange(0, 3).colRange(0, 3), cv::Size(cols_r, rows_r), CV_32F, igb.M1r, igb.M2r);


    ros::NodeHandle nh;

    //pub_odom = nh.advertise<nav_msgs::Odometry>("pose", 1);

    message_filters::Subscriber<sensor_msgs::Image> left_sub(nh, "/zed/left/image_rect_color", 1);
    message_filters::Subscriber<sensor_msgs::Image> right_sub(nh, "/zed/right/image_rect_color", 1);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), left_sub, right_sub);
    sync.registerCallback(boost::bind(&ImageGrabber::GrabStereo, &igb, _1, _2));



    ros::spin();

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory_TUM_Format.txt");
    SLAM.SaveTrajectoryTUM("FrameTrajectory_TUM_Format.txt");
    SLAM.SaveTrajectoryKITTI("FrameTrajectory_KITTI_Format.txt");

    ros::shutdown();

    return 0;
}




cv::Mat undistored(cv_ptrLeft->image, ){

}

void ImageGrabber::GrabStereo(const sensor_msgs::ImageConstPtr &msgLeft, const sensor_msgs::ImageConstPtr &msgRight)
{

    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptrLeft;
    try
    {
        cv_ptrLeft = cv_bridge::toCvShare(msgLeft);
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv_bridge::CvImageConstPtr cv_ptrRight;
    try
    {
        cv_ptrRight = cv_bridge::toCvShare(msgRight);
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv::Mat pose;


//        cv::Mat imLeft, imRight;
//        cv::remap(cv_ptrLeft->image, imLeft, M1l, M2l, cv::INTER_LINEAR);
//        cv::remap(cv_ptrRight->image, imRight, M1r, M2r, cv::INTER_LINEAR);
//        pose = mpSLAM->TrackStereo(imLeft, imRight, cv_ptrLeft->header.stamp.toSec());

    cv::Mat imLeft, imRight;

    imLeft = undistored(cv_ptrLeft->image,);
    imRight = undistored(cv_ptrRight->image,);

    pose = mpSLAM->TrackStereo(imLeft, imRight, cv_ptrLeft->header.stamp.toSec());






    if (pose.empty())
        return;

    /* global left handed coordinate system */
    static cv::Mat pose_prev = cv::Mat::eye(4, 4, CV_32F);
    static cv::Mat world_lh = cv::Mat::eye(4, 4, CV_32F);
    // matrix to flip signs of sinus in rotation matrix, not sure why we need to do that
    static const cv::Mat flipSign = (cv::Mat_<float>(4, 4) << 1, -1, -1, 1,
            -1, 1, -1, 1,
            -1, -1, 1, 1,
            1, 1, 1, 1);

    //prev_pose * T = pose
    cv::Mat translation = (pose * pose_prev.inv()).mul(flipSign);
    world_lh = world_lh * translation;
    pose_prev = pose.clone();

    /* transform into global right handed coordinate system, publish in ROS*/
    tf::Matrix3x3 cameraRotation_rh(-world_lh.at<float>(0, 0), world_lh.at<float>(0, 1), world_lh.at<float>(0, 2),
                                    -world_lh.at<float>(1, 0), world_lh.at<float>(1, 1), world_lh.at<float>(1, 2),
                                    world_lh.at<float>(2, 0), -world_lh.at<float>(2, 1), -world_lh.at<float>(2, 2));

    tf::Vector3 cameraTranslation_rh(world_lh.at<float>(0, 3), world_lh.at<float>(1, 3), -world_lh.at<float>(2, 3));

    //rotate 270deg about x and 270deg about x to get ENU: x forward, y left, z up
    const tf::Matrix3x3 rotation270degXZ(0, 1, 0,
                                         0, 0, 1,
                                         1, 0, 0);

    static tf::TransformBroadcaster broadcaster;

    tf::Matrix3x3 globalRotation_rh = cameraRotation_rh * rotation270degXZ;
    tf::Quaternion q;
    globalRotation_rh.getRotation(q);
    double qNorm = sqrt(q.x() * q.x() + q.y() * q.y() + q.z() * q.z() + q.w() * q.w());
    tf::Quaternion globalQ(q.x() / qNorm, q.y() / qNorm, q.z() / qNorm, q.w() / qNorm);

    tf::Vector3 globalTranslation_rh = cameraTranslation_rh * rotation270degXZ;
    tf::Transform transform = tf::Transform(globalQ, globalTranslation_rh);

    broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "camera_link", "pose"));
}
