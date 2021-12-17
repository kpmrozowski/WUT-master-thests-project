#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <kdl/tree.hpp>
#include <robot_state_publisher/robot_state_publisher.h>
#include <kdl_parser/kdl_parser.hpp>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/Imu.h>
#include <cmath>
#include <stdio.h>
#include <map>
#include <string>

#include <control_toolbox/pid.h>
#include "pid.h"

template <typename T> T sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

# define str(x) std::to_string(x)

class PanporterMotion {
public:
  PanporterMotion(const KDL::Tree& tree, const double publish_frequency);
private:
  void callbackJointState(const sensor_msgs::JointStateConstPtr& state);
  void imuCallback(const sensor_msgs::Imu::ConstPtr& msg);
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
  void velCallback(const geometry_msgs::Twist::ConstPtr& vel);
  ros::Rate publish_rate_;
  robot_state_publisher::RobotStatePublisher state_publisher_;
  const double robot_width_ = 0.4;
  const double wheel_radious_ = 0.35;
  ros::NodeHandle nh_, nh_private_;
  ros::Publisher pub_wheel_left_, pub_wheel_right_, pub_prism_, pub_debug_joy_, pub_debug_imu_, pub_debug_js_;
  ros::Subscriber joy_sub_, imu_sub_, joint_state_sub_;
	// float vel_angular_z_;
	float vel_linear_x_;
  float max_linear_vel_ = 5;
  float max_angular_vel_ = 2 * M_PI;
  float max_m_vel_ = 4.;
  std::size_t axis_count_;
  control_toolbox::Pid pid_m_, pid_lr_;
  bool pid_antiwindup_ = false;
  double angle_error_;
  ros::Time cmd_last_time_;
  double pid_cmd_m_= 0.0, pid_cmd_lr_ = 0.0;
  double joy_cmd_m_= 0.0, joy_cmd_l_ = 0.0, joy_cmd_r_ = 0.0;
  double angle_state_ = 0.0, angle_desired_state_ = 0.0;
  double m_state_ = 0.0, l_state_ = 0.0, r_state_ = 0.0;
  int counter1_ = 0;
  int counter1_limit_ = 100;
  double pid_m_p_= -10.,  pid_m_i_= -1*1e-5,  pid_m_d_= -5*1e3,  pid_m_imax_= 1e5,  pid_m_imin_= -1e5;
  double pid_lr_p_= -10., pid_lr_i_= -1*1e-5, pid_lr_d_= -5*1e3, pid_lr_imax_= 1e5, pid_lr_imin_= -1e5;
};

PanporterMotion::PanporterMotion(const KDL::Tree& tree, const double publish_frequency) : state_publisher_(tree), publish_rate_(publish_frequency)
{
  // vel_sub_ = nh_.subscribe("cmd_vel", 10, &PanporterMotion::velCallback, this);
  joint_state_sub_ = nh_.subscribe<sensor_msgs::JointState>("/panporter1/joint_states", 1, &PanporterMotion::callbackJointState, this);
	imu_sub_ = nh_.subscribe<sensor_msgs::Imu>("/panporter1/imu", 100, &PanporterMotion::imuCallback, this);
	joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 100, &PanporterMotion::joyCallback, this);
  pub_wheel_left_  = nh_.advertise<std_msgs::Float64>("/panporter1/wheel_left_joint_velocity_controller/command", 100);
  pub_wheel_right_ = nh_.advertise<std_msgs::Float64>("/panporter1/wheel_right_joint_velocity_controller/command", 100);
  pub_prism_  = nh_.advertise<std_msgs::Float64>("/panporter1/prism_joint_position_controller/command", 100);
  pub_debug_joy_ = nh_.advertise<std_msgs::String>("/panporter1/debug_joy", 100);
  pub_debug_imu_ = nh_.advertise<std_msgs::String>("/panporter1/debug_imu", 100);
  pub_debug_js_ = nh_.advertise<std_msgs::String>("/panporter1/debug_js", 100);
  double publish_freq;
  if (nh_private_.param("publish_frequency", publish_freq, publish_frequency)) return;
  publish_rate_ = ros::Rate(publish_freq);
  pid_m_ =  control_toolbox::Pid(pid_m_p_,  pid_m_i_,  pid_m_d_,  pid_m_imax_,  pid_m_imin_);
  pid_lr_ = control_toolbox::Pid(pid_lr_p_, pid_lr_i_, pid_lr_d_, pid_lr_imax_, pid_lr_imin_);
}
// -1e-3  |  prism_joint=-0.44  |  angle_state_=0.011  |  ie_m - rising
void PanporterMotion::callbackJointState(const sensor_msgs::JointStateConstPtr& state) {
  if (state->name.size() != state->position.size()){
    ROS_ERROR("Robot state publisher received an invalid joint state vector");
    return;
  }
  
  // get joint positions from state message
  std::map<std::string, double> joint_positions;
  for (unsigned int i = 0; i < state->name.size(); ++i)
    joint_positions.insert(make_pair(state->name[i], state->position[i]));
  // state_publisher_.publishTransforms(joint_positions, state->header.stamp, "panporter1-");
  m_state_ = state->position[0];
  l_state_ = state->position[1];
  r_state_ = state->position[2];
  
  // debug
  std_msgs::String debug_msg;
  debug_msg.data = "";
  for (unsigned int i = 0; i < state->name.size(); ++i)
    debug_msg.data += state->name[i] + "=" + str(state->position[i]) + ", ";
  pub_debug_js_.publish(debug_msg);
  
  publish_rate_.sleep();
}

void PanporterMotion::imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
  ROS_INFO("Imu Orientation x: [%f], y: [%f], z: [%f], w: [%f]", 
    msg->orientation.x,
    msg->orientation.y,
    msg->orientation.z,
    msg->orientation.w
  );
  angle_state_ = msg->orientation.y;
  if (angle_state_ > 0.12 || angle_state_ < -0.12) angle_state_ = angle_desired_state_;
  
  // I1 - integral max, I2 - integrtal min
  // https://docs.ros.org/en/diamondback/api/control_toolbox/html/classcontrol__toolbox_1_1Pid.html
  // https://github.com/awesomebytes/control_toolbox_pid_tutorial
  ros::Time tnow = ros::Time::now();
  angle_error_ = angle_state_ - angle_desired_state_;
  ros::Duration dt = tnow - cmd_last_time_;
  pid_cmd_m_ = 1. * pid_m_.computeCommand(angle_error_, dt);
  pid_cmd_lr_ = 1. * pid_lr_.computeCommand(angle_error_, dt);
  // pid_cmd_m_ = sgn(pid_cmd_m_) * pow(abs(pid_cmd_m_), 0.25);
  // pid_cmd_lr_ = sgn(pid_cmd_m_) * pow(abs(pid_cmd_m_), 0.25);
  
  
  // if (angle_state_ < 0.02 && angle_state_ > -0.02 && ) (angle_state_ < M_PI / 2) {
  // if (abs(joy_cmd_m_) < 0.01) joy_cmd_m_ = - 10. * m_state_;
  std_msgs::Float64 wheel_left_vel_msg;
  std_msgs::Float64 wheel_right_vel_msg;
  std_msgs::Float64 prism_pos_msg;
  wheel_left_vel_msg.data = joy_cmd_l_;// + pid_cmd_lr_;
  wheel_right_vel_msg.data = joy_cmd_r_;// + pid_cmd_lr_;
  prism_pos_msg.data = joy_cmd_m_;// + pid_cmd_m_;
  
  // if (counter1_ > counter1_limit_) {
  //   counter1_=0;
  //   joy_cmd_l_ = 0.0; joy_cmd_r_ = 0.0;
  // }
  // ++counter1_;
  pub_wheel_left_.publish(wheel_left_vel_msg);
  pub_wheel_right_.publish(wheel_right_vel_msg);
  pub_prism_.publish(prism_pos_msg);
  
  // debug
  double pe_m,  ie_m,  de_m;
  double pe_lr, ie_lr, de_lr;
  pid_m_.getCurrentPIDErrors(&pe_m, &ie_m, &de_m);
  pid_lr_.getCurrentPIDErrors(&pe_lr, &ie_lr, &de_lr);
  std_msgs::String debug_msg;
  debug_msg.data = "pid_cmd_m_ ="  + str(pid_cmd_m_) + ", pe_m ="  + str(pid_m_p_ * pe_m)  + ", ie_m ="  + str(pid_m_i_ * ie_m)  + ", de_m =" + str(pid_m_d_ * de_m)    + ",                  "
                 + "pid_cmd_lr_=" + str(pid_cmd_lr_) + ", pe_lr=" + str(pid_lr_p_ * pe_lr) + ", ie_lr=" + str(pid_lr_i_ * ie_lr) + ", de_lr=" + str(pid_lr_d_ * de_lr)
                 + ", angle_state_=" + str(angle_state_);
  pub_debug_imu_.publish(debug_msg);
}

void PanporterMotion::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
	double vel_angular_z = joy->axes[0];
	vel_linear_x_ = joy->axes[1];
  joy_cmd_m_ = max_m_vel_ * joy->axes[4];
  if      (joy->buttons[0]) pid_m_p_ *= 1.5;
  else if (joy->buttons[1]) pid_m_p_ /= 1.5;
  else if (joy->buttons[2]) pid_m_i_ *= 1.5;
  else if (joy->buttons[3]) pid_m_i_ /= 1.5;
  else if (joy->buttons[4]) pid_m_d_ *= 1.5;
  else if (joy->buttons[5]) pid_m_d_ /= 1.5;
  else if (joy->buttons[6]) pid_m_imax_ *= 1.5;
  else if (joy->buttons[7]) pid_m_imax_ /= 1.5;
  else if (joy->buttons[8]) pid_m_imin_ *= 1.5;
  else if (joy->buttons[9]) pid_m_imin_ /= 1.5;
  pid_m_.setGains(pid_m_p_, pid_m_i_, pid_m_d_, pid_m_imax_, pid_m_imin_, pid_antiwindup_);
  
  // prism_pos_msg.data = -prism_pos / 2.;
  joy_cmd_l_ = (max_linear_vel_ * vel_linear_x_ - max_angular_vel_ * vel_angular_z * robot_width_ / 2.) / wheel_radious_;
  joy_cmd_r_ = (max_linear_vel_ * vel_linear_x_ + max_angular_vel_ * vel_angular_z * robot_width_ / 2.) / wheel_radious_;
  
  // debug
  std_msgs::String debug_msg;
  // debug_msg.data = "buttons: ";
  // for (int i = 0; i < joy->buttons.size(); ++i)
  //   debug_msg.data += "b" + str(i) + "=" + str(joy->buttons[i]) + ",";
  debug_msg.data = "\np=" + str(pid_m_p_) + ",i=" + str(pid_m_i_*1e5) + ",d=" + str(pid_m_d_*1e-3) + ",Imax=" + str(pid_m_imax_*1e-5) + ",Imin=" + str(pid_m_imin_*1e-5);
  pub_debug_joy_.publish(debug_msg);
}

void PanporterMotion::velCallback(const geometry_msgs::Twist::ConstPtr& vel)
{
  // linear: <-10,10>
  // angular <-2pi, 2pi>
  std_msgs::Float64 wheel_left_vel_msg;
  std_msgs::Float64 wheel_right_vel_msg;
  std_msgs::Float64 prism_pos;
  wheel_left_vel_msg.data  = (vel->linear.x - 10 * vel->angular.z * robot_width_ / 2);
  wheel_right_vel_msg.data = (vel->linear.x + 10 * vel->angular.z * robot_width_ / 2);
  prism_pos.data = 0.0;
  pub_wheel_left_.publish(wheel_left_vel_msg);
  pub_wheel_right_.publish(wheel_right_vel_msg);
  pub_prism_.publish(prism_pos);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "my_motion");
  
  // gets the location of the robot description on the parameter server
  KDL::Tree tree;
  if (!kdl_parser::treeFromParam("robot_description", tree)) {
    ROS_ERROR("Failed to extract kdl tree from xml robot description");
    return -1;
  }
  if (tree.getNrOfSegments() == 0){
    ROS_WARN("Robot state publisher got an empty tree and cannot publish any state to tf");
    return -1;
    ros::spin();
  } else {
    double freq = 10;
    ros::Rate loop_rate(freq);
    PanporterMotion motion(tree, freq);

    ros::spin();;
  }
  
  // ros::Rate loop_rate(10);
  // loop_rate.sleep();

  return 0;
}

