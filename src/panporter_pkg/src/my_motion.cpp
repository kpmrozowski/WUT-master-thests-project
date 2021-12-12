#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Joy.h>
#include <ros/console.h>
#include <cmath>

using namespace ros::console::levels;

class PanporterMotion {
public:
  PanporterMotion();
private:
  void velCallback(const geometry_msgs::Twist::ConstPtr& vel);
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
  const double robot_width_ = 0.4;
  const double wheel_radious_ = 0.35;
  ros::NodeHandle nh;
  ros::Publisher pub_wheel_left, pub_wheel_right, pub_prism, pub_debug;
  ros::Subscriber sub_;
  float max_linear_vel_ = 10;
  float max_angular_vel_ = 1.5707*4;
  std::size_t axis_count_;
};

PanporterMotion::PanporterMotion()
{
  // sub_ = nh.subscribe("cmd_vel", 10, &PanporterMotion::velCallback, this);
	sub_ = nh.subscribe<sensor_msgs::Joy>("joy", 100, &PanporterMotion::joyCallback, this);
  pub_wheel_left  = nh.advertise<std_msgs::Float64>("/panporter/wheel_left_joint_velocity_controller/command", 100);
  pub_wheel_right = nh.advertise<std_msgs::Float64>("/panporter/wheel_right_joint_velocity_controller/command", 100);
  pub_prism  = nh.advertise<std_msgs::Float64>("/panporter/prism_joint_position_controller/command", 100);
  pub_debug = nh.advertise<std_msgs::String>("/panporter/debug", 100);
}

void PanporterMotion::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
  std_msgs::Float64 wheel_left_vel_msg;
  std_msgs::Float64 wheel_right_vel_msg;
  std_msgs::Float64 prism_pos_msg;
  std_msgs::String debug_msg;
	float vel_angular_z = joy->axes[0];
	float vel_linear_x = joy->axes[1];
  float prism_pos = joy->axes[4];
  debug_msg.data = "joy->axes.size()=" + std::to_string(joy->axes.size());
  // for (int i = 2; i < joy->axes.size(); ++i) {
  //   prism_pos_val += joy->axes[i];
  // }

  wheel_left_vel_msg.data = 40 * (max_linear_vel_ * vel_linear_x - max_angular_vel_ * vel_angular_z * robot_width_ / 2.) / wheel_radious_;
  wheel_right_vel_msg.data = 40. * (max_linear_vel_ * vel_linear_x + max_angular_vel_ * vel_angular_z * robot_width_ / 2.) / wheel_radious_;
  prism_pos_msg.data = -prism_pos / 2.;
  pub_wheel_left.publish(wheel_left_vel_msg);
  pub_wheel_right.publish(wheel_right_vel_msg);
  pub_prism.publish(prism_pos_msg);
  pub_debug.publish(debug_msg);
}

void PanporterMotion::velCallback(const geometry_msgs::Twist::ConstPtr& vel)
{
  // linear: <-10,10>
  // angular <-2pi, 2pi>
  std_msgs::Float64 wheel_left_vel_msg;
  std_msgs::Float64 wheel_right_vel_msg;
  std_msgs::Float64 prism_pos;
  wheel_left_vel_msg.data = 200 * (vel->linear.x - 10 * vel->angular.z * robot_width_ / 2) ;
  wheel_right_vel_msg.data = 200 * (vel->linear.x + 10 * vel->angular.z * robot_width_ / 2);
  prism_pos.data = 0.0;
  pub_wheel_left.publish(wheel_left_vel_msg);
  pub_wheel_right.publish(wheel_right_vel_msg);
  pub_prism.publish(prism_pos);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "my_motion");
  PanporterMotion motion;
  ros::Rate loop_rate(10);

  ros::spin();
  // ros::Rate loop_rate(10);
  // loop_rate.sleep();

  return 0;
}

