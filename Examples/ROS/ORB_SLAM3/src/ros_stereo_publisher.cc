// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/string.hpp"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <fstream>

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class StereoPublisher : public rclcpp::Node
{
public:
  StereoPublisher(std::string left_frame_path, std::string right_frame_path, std::string frame_time)
  : Node("stereo_publisher"), 
  count_(0), 
  timestampe_(0), 
  left_path(left_frame_path), 
  right_path(right_frame_path), 
  frame_time_path(frame_time)
  {
    LoadImages(frame_time_path, frame_names, frame_times);
    publisher_left_ = this->create_publisher<sensor_msgs::msg::Image>("/camera/left/image_raw", 10);
    publisher_right_ = this->create_publisher<sensor_msgs::msg::Image>("/camera/right/image_raw", 10);
    timer_ = this->create_wall_timer(
      50ms, std::bind(&StereoPublisher::timer_callback, this));
  }

private:
  void timer_callback()
  {
    cv::Mat left_mat = cv::imread(left_path + "/" + frame_names[count_], cv::IMREAD_UNCHANGED);
    cv::Mat right_mat = cv::imread(right_path + "/" + frame_names[count_], cv::IMREAD_UNCHANGED);
    sensor_msgs::msg::Image::SharedPtr msg_left = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", left_mat).toImageMsg();
    sensor_msgs::msg::Image::SharedPtr msg_right = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", right_mat).toImageMsg();
    msg_left->header.stamp = this->now();
    msg_right->header.stamp = msg_left->header.stamp;
    RCLCPP_INFO(this->get_logger(), "Publishing left image: size: [%d, %d], time: %d\n", left_mat.rows, left_mat.cols, msg_left->header.stamp.nanosec);
    RCLCPP_INFO(this->get_logger(), "Publishing right image: size: [%d, %d], time: %d\n", right_mat.rows, right_mat.cols, msg_left->header.stamp.nanosec);
    publisher_left_->publish(*msg_left.get());
    publisher_right_->publish(*msg_right.get());
    count_++;
    // timestampe_ += 30000000;
  }
  void LoadImages(const std::string &strPathTimes, std::vector<std::string> &vstrImage, std::vector<long> &vTimeStamps)
  {
    std::ifstream fTimes;
    std::cout << strPathTimes << std::endl;
    fTimes.open(strPathTimes.c_str(), std::ifstream::in);
    vTimeStamps.reserve(5000);
    vstrImage.reserve(5000);
    while(!fTimes.eof())
    {
        std::string s;
        getline(fTimes,s);

        if(!s.empty())
        {
            if (s[0] == '#')
                continue;
            int pos = s.find(' ');
            std::string item = s.substr(0, pos);

            vstrImage.push_back(item + ".png");

            // long long t = std::stoll(item);
            vTimeStamps.push_back(timestampe_);
            timestampe_ += 30000000;
        }
    }
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_left_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_right_;
  size_t count_;
  long timestampe_;
  std::string left_path;
  std::string right_path;
  std::string frame_time_path;
  std::vector<std::string> frame_names;
  std::vector<long> frame_times;
};

int main(int argc, char * argv[])
{
  if(argc != 4)
  {
      std::cerr << std::endl << "Usage: ros2 run orbslam3 StereoPublisher path_to_cam0 path_to_cam1 path_to_frame_name" << std::endl;
      rclcpp::shutdown();
      return 1;
  } 
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<StereoPublisher>(argv[1], argv[2], argv[3]));
  rclcpp::shutdown();
  return 0;
}
