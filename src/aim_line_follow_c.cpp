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
#include <memory>
#include <string>
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

// ROS2 message imports
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "std_msgs/msg/float64.hpp"
#include "nxp_cup_interfaces/msg/pixy_vector.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class LineFollower : public rclcpp::Node
{
public:
  LineFollower()
  : Node("aim_line_follow_c"), count_(0)
  {
    // Delcare and get parameter values
    this->declare_parameter<double>("start_delay", 15.0);
    this->declare_parameter<std::string>("camera_vector_topic", "cupcar0/PixyVector");
    this->declare_parameter<double>("linear_velocity", 1.25);
    this->declare_parameter<double>("angular_velocity", 1.5);
    this->declare_parameter<double>("single_line_steer_scale", 0.5);
    this->get_parameter("start_delay", start_delay_);
    this->get_parameter("camera_vector_topic", camera_vector_topic_);
    this->get_parameter("linear_velocity", linear_velocity_);
    this->get_parameter("angular_velocity", angular_velocity_);
    this->get_parameter("single_line_steer_scale", single_line_steer_scale_);

    // Create publisher and subscribers
    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cupcar0/cmd_vel", 10);
    subscription_ = this->create_subscription<nxp_cup_interfaces::msg::PixyVector>(
                          "/cupcar0/PixyVector", 10, std::bind(&LineFollower::topic_callback,
                          this, std::placeholders::_1));

    sleep(start_delay_);
  }

private:
  int get_num_vectors(nxp_cup_interfaces::msg::PixyVector::SharedPtr msg) const
  {
    int num_vectors = 0;
    if(!(msg->m0_x0 == 0 && msg->m0_x1 == 0 && msg->m0_y0 == 0 && msg->m0_y1 == 0))
    {
      num_vectors++;
    }
    if(!(msg->m1_x0 == 0 && msg->m1_x1 == 0 && msg->m1_y0 == 0 && msg->m1_y1 == 0))
    {
      num_vectors++;
    }
    return num_vectors;
  }

  void topic_callback(const nxp_cup_interfaces::msg::PixyVector::SharedPtr msg) const
  {
    double frame_width = 72;
    double frame_height = 52;

    double window_center = frame_width / 2;

    double x = 0;
    double y = 0;

    double steer = 0;
    double speed = 0;

    geometry_msgs::msg::Vector3 speed_vector;
    geometry_msgs::msg::Vector3 steer_vector;
    geometry_msgs::msg::Twist cmd_vel;

    int num_vectors = get_num_vectors(msg);

    switch(num_vectors)
    {
      case 0:
        speed=0;
        steer=0;
        break;
      case 1:
        if(msg->m0_x1 > msg->m0_x0)
        {
          x = (msg->m0_x1 - msg->m0_x0) / frame_width;
          y = (msg->m0_y1 - msg->m0_y0) / frame_height;
        }
        else
        {
          x = (msg->m0_x0 - msg->m0_x1) / frame_width;
          y = (msg->m0_y0 - msg->m0_y1) / frame_height;
        }

        if((msg->m0_x0 != msg->m0_x1) && (y != 0))
        {
          steer = -angular_velocity_ * (x/y) * single_line_steer_scale_;
        }
        else
        {
          steer = 0;
        }
        speed = linear_velocity_;
        break;
      case 2:
        steer = angular_velocity_ * (((msg->m0_x1 + msg->m1_x1) / 2) - window_center) / frame_width;
        speed = linear_velocity_;
        break;
    }

    speed_vector.x = (speed * (1-abs(2*steer)));
    steer_vector.z = steer;
    
    cmd_vel.linear = speed_vector;
    cmd_vel.angular = steer_vector;

    publisher_->publish(cmd_vel);
  }

  // Parameters
  double start_delay_;
  std::string camera_vector_topic_;
  double linear_velocity_;
  double angular_velocity_;
  double single_line_steer_scale_;
  size_t count_;

  // Pub/sub
  rclcpp::Subscription<nxp_cup_interfaces::msg::PixyVector>::SharedPtr subscription_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LineFollower>());
  rclcpp::shutdown();
  return 0;
}
