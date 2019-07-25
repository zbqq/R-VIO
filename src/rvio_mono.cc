/**
* This file is part of R-VIO.
*
* Copyright (C) 2019 Zheng Huai <zhuai@udel.edu> and Guoquan Huang <ghuang@udel.edu>
* For more information see <http://github.com/rpng/R-VIO> 
*
* R-VIO is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* R-VIO is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with R-VIO. If not, see <http://www.gnu.org/licenses/>.
*/

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/TransformStamped.h>
#include <cv_bridge/cv_bridge.h>
#include <eigen_conversions/eigen_msg.h>

#include <opencv2/core/core.hpp>

#include "rvio/System.h"

ros::Publisher  GtPub; 

class ImageGrabber
{
public:
    ImageGrabber(RVIO::System* pSys) : mpSys(pSys) {}

    void GrabImage(const sensor_msgs::ImageConstPtr& msg);

    RVIO::System* mpSys;
};


class ImuGrabber
{
public:
    ImuGrabber(RVIO::System* pSys) : mpSys(pSys) {}

    void GrabImu(const sensor_msgs::ImuConstPtr& msg);

    RVIO::System* mpSys;
};


class GtGrabber
{
public:
    GtGrabber(RVIO::System* pSys) : mpSys(pSys) {}

    void GrabGt(const geometry_msgs::TransformStampedConstPtr& msg);

    RVIO::System* mpSys;
};

void ImageGrabber::GrabImage(const sensor_msgs::ImageConstPtr& msg)
{
    static int lastseq = -1;
    if ((int)msg->header.seq!=lastseq+1 && lastseq!=-1)
        ROS_ERROR("Image message drop! curr seq: %d expected seq: %d.", msg->header.seq, lastseq+1);
    lastseq = msg->header.seq;

    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::MONO8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    mpSys->MonoVIO(cv_ptr->image, cv_ptr->header.stamp.toSec(), msg->header.seq);
}


void ImuGrabber::GrabImu(const sensor_msgs::ImuConstPtr& msg)
{
    static int lastseq = -1;
    if ((int) msg->header.seq!=lastseq+1 && lastseq!=-1)
        ROS_ERROR("IMU message drop! curr seq: %d expected seq: %d.", msg->header.seq, lastseq+1);
    lastseq = msg->header.seq;

    Eigen::Vector3d angular_velocity;
    tf::vectorMsgToEigen(msg->angular_velocity, angular_velocity);

    Eigen::Vector3d linear_acceleration;
    tf::vectorMsgToEigen(msg->linear_acceleration, linear_acceleration);

    double currtime = msg->header.stamp.toSec();

    RVIO::ImuData* pData = new RVIO::ImuData();
    pData->AngularVel = angular_velocity;
    pData->LinearAccel = linear_acceleration;
    pData->Timestamp = currtime;

    static double lasttime = -1;
    if (lasttime!=-1)
        pData->TimeInterval = currtime-lasttime;
    else
        pData->TimeInterval = 0;
    lasttime = currtime;

    mpSys->PushImuData(pData);
}

void GtGrabber::GrabGt(const geometry_msgs::TransformStampedConstPtr& msg)
{
    geometry_msgs::PoseStamped pose;
    pose.header.stamp = ros::Time::now();
    pose.header.frame_id = "world";
    pose.pose.position.x = msg->transform.translation.x;
    pose.pose.position.y = msg->transform.translation.y;
    pose.pose.position.z = msg->transform.translation.z;
    pose.pose.orientation.x = msg->transform.rotation.x;
    pose.pose.orientation.y = msg->transform.rotation.y;
    pose.pose.orientation.z = msg->transform.rotation.z;
    pose.pose.orientation.w = msg->transform.rotation.w;
    GtPub.publish(pose);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rvio");
    ros::start();

    RVIO::System Sys(argv[1]);

    ImageGrabber igb1(&Sys);
    ImuGrabber igb2(&Sys);
    GtGrabber igb3(&Sys);

    ros::NodeHandle nodeHandler;
    ros::Subscriber image_sub = nodeHandler.subscribe("/camera/image_raw", 1000, &ImageGrabber::GrabImage, &igb1);
    ros::Subscriber imu_sub = nodeHandler.subscribe("/imu", 10000, &ImuGrabber::GrabImu, &igb2);
    ros::Subscriber gt_sub = nodeHandler.subscribe("/vicon/firefly_sbx/firefly_sbx", 100, &GtGrabber::GrabGt, &igb3);
    GtPub = nodeHandler.advertise<geometry_msgs::PoseStamped>("/rvio/gt", 1);

    ros::spin();

    ros::shutdown();

    return 0;
}
