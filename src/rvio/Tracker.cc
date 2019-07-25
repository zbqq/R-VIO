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

#include <algorithm> // std::copy
#include <iterator> // std::back_inserter

#include <Eigen/Core>

#include <opencv2/core/eigen.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include <sensor_msgs/Image.h>

#include "rvio/Tracker.h"
#include "numerics.h"


namespace RVIO
{

CvScalar red = CV_RGB(255,64,64);
CvScalar green = CV_RGB(64,255,64);
CvScalar blue = CV_RGB(64,64,255);

Tracker::Tracker(const std::string& strSettingsFile)
{
    // Read settings file
    cv::FileStorage fsSettings(strSettingsFile, cv::FileStorage::READ);

    const int nImageWidth = fsSettings["Camera.width"];
    const int nImageHeight = fsSettings["Camera.height"];

    const float fx = fsSettings["Camera.fx"];
    const float fy = fsSettings["Camera.fy"];
    const float cx = fsSettings["Camera.cx"];
    const float cy = fsSettings["Camera.cy"];

    cv::Mat K = cv::Mat::eye(3,3,CV_32F);
    K.at<float>(0,0) = fx;
    K.at<float>(1,1) = fy;
    K.at<float>(0,2) = cx;
    K.at<float>(1,2) = cy;
    K.copyTo(mK);

    cv::Mat DistCoef(4,1,CV_32F);
    DistCoef.at<float>(0) = fsSettings["Camera.k1"];
    DistCoef.at<float>(1) = fsSettings["Camera.k2"];
    DistCoef.at<float>(2) = fsSettings["Camera.p1"];
    DistCoef.at<float>(3) = fsSettings["Camera.p2"];
    const float k3 = fsSettings["Camera.k3"];
    if(k3!=0)
    {
        DistCoef.resize(5);
        DistCoef.at<float>(4) = k3;
    }
    DistCoef.copyTo(mDistCoef);

    cv::Mat T(4,4,CV_32F);
    fsSettings["Camera.T_BC0"] >> T;
    Eigen::Matrix4d Tic;
    cv::cv2eigen(T,Tic);
    mRic = Tic.block<3,3>(0,0);
    mtic = Tic.block<3,1>(0,3);
    mRci = mRic.transpose();
    mtci = -mRci*mtic;

    mnSmallAngle = fsSettings["IMU.nSmallAngle"];

    const int bIsRGB = fsSettings["Camera.RGB"];
    mbIsRGB = bIsRGB;

    const int bIsFisheye = fsSettings["Camera.Fisheye"];
    mbIsFisheye = bIsFisheye;

    const int bEnableEqualizer = fsSettings["Tracker.EnableEqualizer"];
    mbEnableEqualizer = bEnableEqualizer;

    mnMaxFeatsPerImage = fsSettings["Tracker.nFeatures"];
    mnMaxFeatsForUpdate = std::ceil(0.5*mnMaxFeatsPerImage);

    mvlTrackingHistory.resize(mnMaxFeatsPerImage);
    mvvPoseHistory.resize(mnMaxFeatsPerImage);

    mnMaxTrackingLength = fsSettings["Tracker.nMaxTrackingLength"];
    mnMinTrackingLength = fsSettings["Tracker.nMinTrackingLength"];

    const double nQualLvl = fsSettings["Tracker.nQualLvl"];
    const double nMinDist = fsSettings["Tracker.nMinDist"];
    const double nGridSize = fsSettings["Tracker.nGridSize"];
    mpCornerDetector = new CornerDetector(nImageHeight, nImageWidth, nQualLvl, nMinDist);
    mpCornerCluster = new CornerCluster(nImageHeight, nImageWidth, nGridSize);

    const int bUseSampson = fsSettings["Tracker.UseSampson"];
    const double nInlierThreshold = fsSettings["Tracker.nSampsonThrd"];
    mpRansac = new Ransac(bUseSampson, nInlierThreshold);

    mbIsTheFirstImage = true;

    mLastImage = cv::Mat();

    mTrackPub = mTrackerNode.advertise<sensor_msgs::Image>("/rvio/track", 2);
    mNewerPub = mTrackerNode.advertise<sensor_msgs::Image>("/rvio/newer", 2);
}


Tracker::~Tracker()
{
    delete mpCornerDetector;
    delete mpCornerCluster;
    delete mpRansac;
}


void Tracker::UndistortAndNormalize(const int N,
                                    std::vector<cv::Point2f>& src,
                                    std::vector<cv::Point2f>& dst)
{
    cv::Mat mat(N,2,CV_32F);

    // Get (u,v) from src
    for(int i=0; i<N; i++)
    {
        mat.at<float>(i,0) = src.at(i).x;
        mat.at<float>(i,1) = src.at(i).y;
    }

    // Undistort and normalize (u,v)
    mat = mat.reshape(2);
    if (!mbIsFisheye)
        cv::undistortPoints(mat, mat, mK, mDistCoef);
    else
        cv::fisheye::undistortPoints(mat, mat, mK, mDistCoef);
    mat = mat.reshape(1);

    // Fill (u'/f,v'/f) to dst
    dst.clear();
    for(int i=0; i<N; ++i)
    {
        cv::Point2f ptUN;
        ptUN.x = mat.at<float>(i,0);
        ptUN.y = mat.at<float>(i,1);

        dst.push_back(ptUN);
    }
}


void Tracker::GetRotation(Eigen::Matrix3d& R,
                          Eigen::VectorXd& xkk,
                          std::list<ImuData*>& plImuData)
{
    Eigen::Matrix3d tempR;
    tempR.setIdentity();

    Eigen::Matrix3d I;
    I.setIdentity();

    Eigen::Vector3d bg = xkk.block(20,0,3,1);

    for (std::list<ImuData*>::const_iterator lit=plImuData.begin();
         lit!=plImuData.end(); ++lit)
    {
        Eigen::Vector3d wm = (*lit)->AngularVel;
        double dt = (*lit)->TimeInterval;

        Eigen::Vector3d w = wm-bg;

        bool bIsSmallAngle = false;
        if (w.norm()<mnSmallAngle)
            bIsSmallAngle = true;

        double w1 = w.norm();
        double wdt = w1*dt;
        Eigen::Matrix3d wx = SkewSymm(w);
        Eigen::Matrix3d wx2 = wx*wx;

        Eigen::Matrix3d deltaR;
        if (bIsSmallAngle)
            deltaR = I-dt*wx+(.5*pow(dt,2))*wx2;
        else
            deltaR = I-(sin(wdt)/w1)*wx+((1-cos(wdt))/pow(w1,2))*wx2;
        assert(std::isnan(deltaR.norm())!=true);

        tempR = deltaR*tempR;
    }

    R = mRci*tempR*mRic;
}


void Tracker::DisplayTrack(const cv::Mat& imIn,
                           std::vector<cv::Point2f>& vPoints1,
                           std::vector<cv::Point2f>& vPoints2,
                           std::vector<unsigned char>& vInlierFlag,
                           cv_bridge::CvImage& imOut)
{
    imOut.header = std_msgs::Header();
    imOut.encoding = "bgr8";

    cvtColor(imIn, imOut.image, CV_GRAY2BGR);

    for (int i=0; i<(int)vPoints1.size(); ++i)
    {
        if (vInlierFlag.at(i)!=0)
        {
            cv::circle(imOut.image, vPoints1.at(i), 3, blue, -1);
            cv::line(imOut.image, vPoints1.at(i), vPoints2.at(i), blue);
        }
        else
        {
            cv::circle(imOut.image, vPoints1.at(i), 3, red, 0);
        }
    }
}


void Tracker::DisplayNewer(const cv::Mat& imIn,
                           std::vector<cv::Point2f>& vFeats,
                           std::vector<cv::Point2f>& vNewFeats,
                           cv_bridge::CvImage& imOut)
{
    imOut.header = std_msgs::Header();
    imOut.encoding = "bgr8";

    cvtColor(imIn, imOut.image, CV_GRAY2BGR);

    for (int i=0; i<(int)vFeats.size(); ++i)
        cv::circle(imOut.image, vFeats.at(i), 3, blue, 0);

    for (int i=0; i<(int)vNewFeats.size(); ++i)
        cv::circle(imOut.image, vNewFeats.at(i), 3, green, -1);
}


void Tracker::track(cv::Mat& im,
                    Eigen::VectorXd& xkk,
                    std::list<ImuData*>& plImuData)
{
    // Convert to gray scale
    if (im.channels()==3)
    {
        if (mbIsRGB)
            cvtColor(im, im, CV_RGB2GRAY);
        else
            cvtColor(im, im, CV_BGR2GRAY);
    }
    else if (im.channels()==4)
    {
        if (mbIsRGB)
            cvtColor(im, im, CV_RGBA2GRAY);
        else
            cvtColor(im, im, CV_BGRA2GRAY);
    }

    if (mbEnableEqualizer)
    {
        cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(3.0, cv::Size(15,15));
        clahe->apply(im, im);
    }

    if (mbIsTheFirstImage)
    {
        // Detect features
        mnFeatsToTrack = mpCornerDetector->DetectWithSubPix(mnMaxFeatsPerImage, im, mvFeatsToTrack);

        if (mnFeatsToTrack==0)
        {
            ROS_DEBUG("No features available to track.");
            return;
        }

        std::vector<cv::Point2f> vFeatsUndistNorm;
        UndistortAndNormalize(mnFeatsToTrack, mvFeatsToTrack, vFeatsUndistNorm);

        mPoints1ForRansac.setZero(3,mnFeatsToTrack);
        for (int i=0; i<mnFeatsToTrack; ++i)
        {
            cv::Point2f ptUN = vFeatsUndistNorm.at(i);
            mvlTrackingHistory.at(i).push_back(ptUN);
            mvvPoseHistory.at(i).push_back(0);
            Eigen::Vector3d ptUNe = Eigen::Vector3d(ptUN.x,ptUN.y,1);
            mPoints1ForRansac.block(0,i,3,1) = ptUNe;

            mvInlierIndices.push_back(i);
        }

        for (int i=mnFeatsToTrack; i<mnMaxFeatsPerImage; ++i)
            mlFreeIndices.push_back(i);

        mbIsTheFirstImage = false;
    }
    else
    {
        cv::Size winSize(15,15);
        cv::TermCriteria termCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 30, 1e-2);
        std::vector<cv::Point2f> vFeatsTracked;
        std::vector<unsigned char> vInlierFlag;
        std::vector<float> vTrackingError;

        // Lucas-Kanade method
        cv::calcOpticalFlowPyrLK(mLastImage, im, mvFeatsToTrack, vFeatsTracked, vInlierFlag, vTrackingError, winSize, 3, termCriteria, 0, 1e-3);

        if (vFeatsTracked.empty())
        {
            ROS_DEBUG("No features tracked in current image.");
            return;
        }

        std::vector<cv::Point2f> vFeatsUndistNorm;
        UndistortAndNormalize(mnFeatsToTrack, vFeatsTracked, vFeatsUndistNorm);

        mPoints2ForRansac.setZero(3,mnFeatsToTrack);
        for (int i=0; i<mnFeatsToTrack; ++i)
        {
            cv::Point2f ptUN = vFeatsUndistNorm.at(i);
            Eigen::Vector3d ptUNe = Eigen::Vector3d(ptUN.x,ptUN.y,1);
            mPoints2ForRansac.block(0,i,3,1) = ptUNe;
        }

        // RANSAC
        Eigen::Matrix3d R;
        GetRotation(R, xkk, plImuData);
        mpRansac->FindInliers(mPoints1ForRansac, mPoints2ForRansac, R.transpose(), vInlierFlag);

        // Show the result in rviz
        cv_bridge::CvImage imTrack;
        DisplayTrack(im, mvFeatsToTrack, vFeatsTracked, vInlierFlag, imTrack);
        mTrackPub.publish(imTrack.toImageMsg());

        // Prepare data for update
        mvFeatTypesForUpdate.clear();
        mvlFeatMeasForUpdate.clear();
        mvvPoseForUpdate.clear();
        mvlFeatMeasForUpdate.resize(mnMaxFeatsForUpdate);
        mvvPoseForUpdate.resize(mnMaxFeatsForUpdate);
        mvFeatsToTrack.clear();
        std::vector<int> vInlierIndicesToTrack;
        Eigen::MatrixXd tempPointsForRansac(3,mnMaxFeatsPerImage);

        int nMeasCount = 0;
        int nInlierCount = 0;

        for (int i=0; i<mnFeatsToTrack; ++i)
        {
            if (!vInlierFlag.at(i))
            {
                // Lose track
                int idx = mvInlierIndices.at(i);
                mlFreeIndices.push_back(idx);

                if ((int)mvlTrackingHistory.at(idx).size()>=mnMinTrackingLength)
                {
                    if (nMeasCount<mnMaxFeatsForUpdate)
                    {
                        mvFeatTypesForUpdate.push_back('1');
                        mvlFeatMeasForUpdate.at(nMeasCount) = mvlTrackingHistory.at(idx);
                        mvvPoseForUpdate.at(nMeasCount) = mvvPoseHistory.at(idx);
                        nMeasCount++;
                    }
                }

                mvlTrackingHistory.at(idx).clear();
                mvvPoseHistory.at(idx).clear();
            }
            else
            {
                // Tracked
                int idx = mvInlierIndices.at(i);
                vInlierIndicesToTrack.push_back(idx);

                cv::Point2f pt = vFeatsTracked.at(i);
                mvFeatsToTrack.push_back(pt);

                cv::Point2f ptUN = vFeatsUndistNorm.at(i);
                if ((int)mvlTrackingHistory.at(idx).size()==mnMaxTrackingLength)
                {
                    // Reach the max. tracking length
                    // Note: We use all measurements for triangulation, while
                    //       Only use the first 1/2 measurements for update.
                    if (nMeasCount<mnMaxFeatsForUpdate)
                    {
                        mvFeatTypesForUpdate.push_back('2');
                        mvlFeatMeasForUpdate.at(nMeasCount) = mvlTrackingHistory.at(idx);
/*
                        while (mvlTrackingHistory.at(idx).size()>10)
                        {
                            mvlTrackingHistory.at(idx).pop_front();
                            mvvPoseForUpdate.at(nMeasCount).push_back(mvvPoseHistory.at(idx)[0]);
                            mvvPoseHistory.at(idx).erase(mvvPoseHistory.at(idx).begin());
                            
                        }
*/

                        auto iter1=mvlTrackingHistory.at(idx).begin(); 
                        auto iter2=mvvPoseHistory.at(idx).begin();    
                        for(int i=0;i<4;i++)
                        {
                            iter1 = mvlTrackingHistory.at(idx).erase(iter1);
                            mvvPoseForUpdate.at(nMeasCount).push_back(*iter2);
                            iter2 = mvvPoseHistory.at(idx).erase(iter2);
                            //++iter1;++iter1;++iter1;
                            //++iter2;++iter2;++iter2;
                        }

                        nMeasCount++;
                    }
                    else
                    {
                        mvlTrackingHistory.at(idx).pop_front();
                        mvvPoseHistory.at(idx).erase(mvvPoseHistory.at(idx).begin());
                        std::cout<<"drop"<<std::endl;
                        
                    }
                        
                }
                mvlTrackingHistory.at(idx).push_back(ptUN);
                if(((xkk.rows()-26)/7)==mnMaxTrackingLength-1)
                {
                    std::for_each(mvvPoseHistory.at(idx).begin(),mvvPoseHistory.at(idx).end(),MinusOne);
                    if(mvvPoseHistory.at(idx)[0]<0)
                    {
                        mvlTrackingHistory.at(idx).pop_front();
                        mvvPoseHistory.at(idx).erase(mvvPoseHistory.at(idx).begin());
                        std::cout<<"drop"<<std::endl;
                    }    
                }
                
                mvvPoseHistory.at(idx).push_back((xkk.rows()-26)/7);
                Eigen::Vector3d ptUNe = Eigen::Vector3d(ptUN.x,ptUN.y,1);
                tempPointsForRansac.block(0,nInlierCount,3,1) = ptUNe;

                nInlierCount++;
            }
        }
/*
        for(auto a:mvvPoseHistory)
        {
            for(auto b:a)
                std::cout<<b<<" ";
            std::cout<<std::endl;
        }

        for(auto a:mvvPoseForUpdate)
        {
            for(auto b:a)
                std::cout<<b<<" ";
            std::cout<<std::endl;
        }

        for(auto a:mvFeatTypesForUpdate)
        {
            std::cout<<a<<std::endl;
        }
*/        
        if (!mlFreeIndices.empty())
        {
            // Feature supplement
            std::vector<cv::Point2f> vTempFeats;
            std::vector<cv::Point2f> vNewFeats;

            mpCornerCluster->ChessGrid(mvFeatsToTrack);
            mpCornerDetector->DetectWithSubPix(mnMaxFeatsPerImage, im, vTempFeats);
            int nNewFeats = mpCornerCluster->FindNew(vTempFeats, vNewFeats, 10);

            // Show the result in rviz
            cv_bridge::CvImage imNewer;
            DisplayNewer(im, vTempFeats, vNewFeats, imNewer);
            mNewerPub.publish(imNewer.toImageMsg());

            if (nNewFeats!=0)
            {
                std::list<cv::Point2f> lNewFeats;
                std::copy(vNewFeats.begin(), vNewFeats.end(), std::back_inserter(lNewFeats));

                std::vector<cv::Point2f> vNewFeatsUndistNorm;
                UndistortAndNormalize(nNewFeats, vNewFeats, vNewFeatsUndistNorm);

                std::list<cv::Point2f> lNewFeatsUndistNorm;
                std::copy(vNewFeatsUndistNorm.begin(), vNewFeatsUndistNorm.end(), std::back_inserter(lNewFeatsUndistNorm));

                for (;;)
                {
                    int idx = mlFreeIndices.front();
                    vInlierIndicesToTrack.push_back(idx);

                    cv::Point2f pt = lNewFeats.front();
                    mvFeatsToTrack.push_back(pt);

                    cv::Point2f ptUN = lNewFeatsUndistNorm.front();
                    mvlTrackingHistory.at(idx).push_back(ptUN);
                    mvvPoseHistory.at(idx).push_back((xkk.rows()-26)/7);
                    Eigen::Vector3d ptUNe = Eigen::Vector3d(ptUN.x,ptUN.y,1);
                    tempPointsForRansac.block(0,nInlierCount,3,1) = ptUNe;

                    nInlierCount++;

                    mlFreeIndices.pop_front();
                    lNewFeats.pop_front();
                    lNewFeatsUndistNorm.pop_front();

                    if (mlFreeIndices.empty() || lNewFeats.empty() || lNewFeatsUndistNorm.empty() || nInlierCount==mnMaxFeatsPerImage)
                        break;
                }
            }
        }

        // Update tracker
        mnFeatsToTrack = nInlierCount;
        mvInlierIndices = vInlierIndicesToTrack;
        mPoints1ForRansac = tempPointsForRansac.block(0,0,3,nInlierCount);
    }

    im.copyTo(mLastImage);
}

} // namespace RVIO
