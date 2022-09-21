/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2020 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include <boost/bind.hpp>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include<opencv2/core/core.hpp>

#include"../../../include/System.h"

using namespace std;

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM3::System* pSLAM): mpSLAM(pSLAM){
        imageScale = mpSLAM->GetImageScale();
    }

    void GrabStereo(const sensor_msgs::msg::Image::ConstSharedPtr& msgLeft,const sensor_msgs::msg::Image::ConstSharedPtr& msgRight);

    ORB_SLAM3::System* mpSLAM;
    float imageScale;
    cv::Mat M1l,M2l,M1r,M2r;
};

class StereoSubscriber : public rclcpp::Node {
    public:
    StereoSubscriber(ImageGrabber *igb) : Node("stereo_subscriber"), igb_(igb) {
        left_sub.subscribe(this, "/camera/left/image_raw");
        right_sub.subscribe(this, "/camera/right/image_raw");
        typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image> sync_pol;
        message_filters::TimeSynchronizer<sensor_msgs::msg::Image, sensor_msgs::msg::Image> sync(left_sub, right_sub, 10);
        //message_filters::Synchronizer<sync_pol> sync(sync_pol(10), left_sub, right_sub);
        sync.registerCallback(boost::bind(&StereoSubscriber::call_back, this));
    }
    private:
    void call_back() { 
        RCLCPP_INFO(this->get_logger(), "I heard: left, right");
    }
    ImageGrabber *igb_;
    message_filters::Subscriber<sensor_msgs::msg::Image> left_sub;
    message_filters::Subscriber<sensor_msgs::msg::Image> right_sub;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    if(argc != 3)
    {
        cerr << endl << "Usage: ros2 run orbslam3 Stereo path_to_vocabulary path_to_settings" << endl;
        rclcpp::shutdown();
        return 1;
    }    

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM3::System SLAM(argv[1],argv[2],ORB_SLAM3::System::STEREO,true);

    ImageGrabber igb(&SLAM);
    rclcpp::spin(std::make_shared<StereoSubscriber>(&igb));

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory_TUM_Format.txt");
    SLAM.SaveTrajectoryTUM("FrameTrajectory_TUM_Format.txt");
    SLAM.SaveTrajectoryKITTI("FrameTrajectory_KITTI_Format.txt");

    rclcpp::shutdown();

    return 0;
}

void ImageGrabber::GrabStereo(const sensor_msgs::msg::Image::ConstSharedPtr& msgLeft,const sensor_msgs::msg::Image::ConstSharedPtr& msgRight)
{
    // Copy the ros image message to cv::Mat.
    printf("I heard: left, right");
    cv_bridge::CvImageConstPtr cv_ptrLeft;
    try
    {
        cv_ptrLeft = cv_bridge::toCvShare(msgLeft);
    }
    catch (cv_bridge::Exception& e)
    {
        printf("cv_bridge exception: %s", e.what());
        return;
    }

    cv_bridge::CvImageConstPtr cv_ptrRight;
    try
    {
        cv_ptrRight = cv_bridge::toCvShare(msgRight);
    }
    catch (cv_bridge::Exception& e)
    {
        printf("cv_bridge exception: %s", e.what());
        return;
    }

    if(imageScale != 1.f)
    {
#ifdef REGISTER_TIMES
    #ifdef COMPILEDWITHC11
                std::chrono::steady_clock::time_point t_Start_Resize = std::chrono::steady_clock::now();
    #else
                std::chrono::monotonic_clock::time_point t_Start_Resize = std::chrono::monotonic_clock::now();
    #endif
#endif
                int width = cv_ptrLeft->image.cols * imageScale;
                int height = cv_ptrLeft->image.rows * imageScale;
                cv::Mat imLeft, imRight;
                cv::resize(cv_ptrLeft->image, imLeft, cv::Size(width, height));
                cv::resize(cv_ptrRight->image, imRight, cv::Size(width, height));
#ifdef REGISTER_TIMES
    #ifdef COMPILEDWITHC11
                std::chrono::steady_clock::time_point t_End_Resize = std::chrono::steady_clock::now();
    #else
                std::chrono::monotonic_clock::time_point t_End_Resize = std::chrono::monotonic_clock::now();
    #endif
                t_resize = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(t_End_Resize - t_Start_Resize).count();
                SLAM.InsertResizeTime(t_resize);
#endif
        printf("TrackStereo: time %f", cv_ptrLeft->header.stamp.sec);
        mpSLAM->TrackStereo(imLeft,imRight,cv_ptrLeft->header.stamp.sec);
    } else {
        printf("TrackStereo: time %f", cv_ptrLeft->header.stamp.sec);
        mpSLAM->TrackStereo(cv_ptrLeft->image,cv_ptrRight->image,cv_ptrLeft->header.stamp.sec);
    }

}
