/* AIM Line Follow Example Code - C++
 *
 * This example code is meant to act as an example for simple line-following control
 * in NXP Cup Summer Camp and NXP AIM Challenge.
 *
 * The code is heavily commented to help students and professors understand the code.
 *
 * Questions? Email landon.haugh@nxp.com
 *
 * Written by: Landon Haugh (landon.haugh@nxp.com)
 *             Aditya Vashista (aditya.vashista@nxp.com)
 */

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

class LineFollower : public rclcpp::Node
{

public:

  LineFollower() : Node("aim_line_follow_c"), count_(0)
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
    pixy_subscription_ = this->create_subscription<nxp_cup_interfaces::msg::PixyVector>(
                          "/cupcar0/PixyVector", 10, std::bind(&LineFollower::listener_callback,
                          this, std::placeholders::_1));
    sleep(start_delay_);
  }

/*
 * NXP SUMMER CAMP PARTICIPANTS:
 * There are two included functions within the private section for your convenience.
 * get_num_vectors() and publish_controls()
 * If you want to add new functions, just add them in the private section.
 */
private:

  // START DO NOT EDIT
  int get_num_vectors(nxp_cup_interfaces::msg::PixyVector::SharedPtr msg)
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

  void publish_controls(double speed, double steer)
  {
    speed_vector_.x = (speed * (1-abs(2*steer)));
    steer_vector_.z = steer;

    cmd_vel_.linear = speed_vector_;
    cmd_vel_.angular = steer_vector_;

    publisher_->publish(cmd_vel_);

    return;
  }
  // END DO NOT EDIT

  /*
   * NXP SUMMER CAMP PARTICIPANTS:
   * - This is the function that you will write your self-driving code in.
   * - Each time new line vector data is found, this function will run.
   * - You can use 'msg->m?_??' to use the line vector data.
   * - Store your calculated speed and steer values in the 'speed' and 'steer' variables.
   * - Use the publish_controls() function to send these values to the simulated car.
   */
  void listener_callback(nxp_cup_interfaces::msg::PixyVector::SharedPtr msg)
  {
    double frame_width = 78;
    double frame_height = 51;
    double window_center = frame_width / 2;

    double x = 0;
    double y = 0;

    double steer = 0;
    double speed = 0;

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

    // Publish speed/steer controls
    publish_controls(speed, steer);
  }

  // Parameters
  double start_delay_;
  std::string camera_vector_topic_;
  double linear_velocity_;
  double angular_velocity_;
  double single_line_steer_scale_;
  size_t count_;

  // Control values
  geometry_msgs::msg::Vector3 speed_vector_;
  geometry_msgs::msg::Vector3 steer_vector_;
  geometry_msgs::msg::Twist cmd_vel_;

  // Pub/sub
  rclcpp::Subscription<nxp_cup_interfaces::msg::PixyVector>::SharedPtr pixy_subscription_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
};

// START DO NOT EDIT
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LineFollower>());
  rclcpp::shutdown();
  return 0;
}
// END DO NOT EDIT
