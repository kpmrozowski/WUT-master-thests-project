#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <kdl/tree.hpp>
// #include <robot_state_publisher/joint_state_listener.h>
#include <robot_state_publisher/robot_state_publisher.h>
#include <kdl_parser/kdl_parser.hpp>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/Imu.h>
#include <cmath>
#include <stdio.h>
#include <map>
#include <string>
#include <fstream>
#include <sstream>
#include <vector>

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
  float max_linear_vel_ = 10;
  float max_angular_vel_ = 1.5707*4;
  std::size_t axis_count_;
  control_toolbox::Pid pid_ros_;
  double pid_p_=0., pid_i_=0., pid_d_=0., pid_imax=0., pid_i_min_=0.;
  bool pid_antiwindup_ = false;
  double error_;
  ros::Time cmd_last_time_;
  double cmd_now_= 0.0, cmd_last_= 0.0;
  double x_state_ = 0.0, x_desired_state_ = 0.0;
  int counter1_ = 0;
  int counter1_limit_ = 100;
};

PanporterMotion::PanporterMotion(const KDL::Tree& tree, const double publish_frequency) : state_publisher_(tree), publish_rate_(publish_frequency)
{
  // vel_sub_ = nh_.subscribe("cmd_vel", 10, &PanporterMotion::velCallback, this);
  joint_state_sub_ = nh_.subscribe<sensor_msgs::JointState>("/panporter/joint_states", 1, &PanporterMotion::callbackJointState, this);
	imu_sub_ = nh_.subscribe<sensor_msgs::Imu>("/panporter/imu", 100, &PanporterMotion::imuCallback, this);
	joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 100, &PanporterMotion::joyCallback, this);
  pub_wheel_left_  = nh_.advertise<std_msgs::Float64>("/panporter/wheel_left_joint_velocity_controller/command", 100);
  pub_wheel_right_ = nh_.advertise<std_msgs::Float64>("/panporter/wheel_right_joint_velocity_controller/command", 100);
  pub_prism_  = nh_.advertise<std_msgs::Float64>("/panporter/prism_joint_position_controller/command", 100);
  pub_debug_joy_ = nh_.advertise<std_msgs::String>("/panporter/debug_joy", 100);
  pub_debug_imu_ = nh_.advertise<std_msgs::String>("/panporter/debug_imu", 100);
  pub_debug_js_ = nh_.advertise<std_msgs::String>("/panporter/debug_js", 100);
  double publish_freq;
  if (nh_private_.param("publish_frequency", publish_freq, publish_frequency)) return;
  publish_rate_ = ros::Rate(publish_freq);
  pid_ros_ = control_toolbox::Pid(- 10., - 1 * 1e-5, - 5 * 1e5, 1e5, -1e5);
}
// -1e-3  |  prism_joint=-0.44  |  x_state_=0.011  |  ie - rising
void PanporterMotion::callbackJointState(const sensor_msgs::JointStateConstPtr& state) {
  if (state->name.size() != state->position.size()){
    ROS_ERROR("Robot state publisher received an invalid joint state vector");
    return;
  }
  
  // get joint positions from state message
  std::map<std::string, double> joint_positions;
  for (unsigned int i = 0; i < state->name.size(); ++i)
    joint_positions.insert(make_pair(state->name[i], state->position[i]));
  // state_publisher_.publishTransforms(joint_positions, state->header.stamp, "panporter-");
  
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
  x_state_ = msg->orientation.y;
  if (x_state_ > 0.12 || x_state_ < -0.12) x_state_ = x_desired_state_;
  
  // I1 - integral max, I2 - integrtal min
  // https://docs.ros.org/en/diamondback/api/control_toolbox/html/classcontrol__toolbox_1_1Pid.html
  // https://github.com/awesomebytes/control_toolbox_pid_tutorial
  double pe, ie, de;
  pid_ros_.getCurrentPIDErrors(&pe, &ie, &de);
  ros::Time tnow = ros::Time::now();
  ros::Duration dt = tnow - cmd_last_time_;
  double error = x_state_ - x_desired_state_;
  
  if (counter1_ > counter1_limit_) {
    // pid_ros_.setGains(pid_p_, pid_i_, pid_d_, pid_imax, pid_i_min_, pid_antiwindup_);
    counter1_=0;
  }
  ++counter1_;
  
  cmd_now_ = 1. * pid_ros_.computeCommand(error, dt);
  cmd_now_ = sgn(cmd_now_) * pow(abs(cmd_now_), 0.5);
  
  // PID pid = PID(0.1, 100, -100, 0.1, 0.01, 0.5);
  // double val = 20;
  // for (int i = 0; i < 100; i++) {
  //     double inc = pid.calculate(0, val);
  //     printf("val:% 7.3f inc:% 7.3f\n", val, inc);
  //     val += inc;
  // }
  
  std_msgs::Float64 prism_pos_msg;
  if (x_state_ < M_PI) {
    prism_pos_msg.data = cmd_now_;
  } else {
    // robot is upside-down...
    prism_pos_msg.data = x_state_;
  }
  pub_prism_.publish(prism_pos_msg);

  // debug
  std_msgs::String debug_msg;
  debug_msg.data = "cmd_now_=" + str(cmd_now_) + ", pe=" + str(- 10. * pe) + ", ie=" + str(-5e-5 * ie) + ", de=" + str(5e5 * de) + ", x_state_=" + str(x_state_);
  pub_debug_imu_.publish(debug_msg);
}

void PanporterMotion::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
  std_msgs::Float64 wheel_left_vel_msg;
  std_msgs::Float64 wheel_right_vel_msg;
  std_msgs::Float64 prism_pos_msg;
	float vel_angular_z = joy->axes[0];
	float vel_linear_x = joy->axes[1];
  // float prism_pos = joy->axes[4];


  wheel_left_vel_msg.data = 40 * (max_linear_vel_ * vel_linear_x - max_angular_vel_ * vel_angular_z * robot_width_ / 2.) / wheel_radious_;
  wheel_right_vel_msg.data = 40. * (max_linear_vel_ * vel_linear_x + max_angular_vel_ * vel_angular_z * robot_width_ / 2.) / wheel_radious_;
  // prism_pos_msg.data = -prism_pos / 2.;
  pub_wheel_left_.publish(wheel_left_vel_msg);
  pub_wheel_right_.publish(wheel_right_vel_msg);
  // pub_prism_.publish(prism_pos_msg);
  
  // debug
  // std_msgs::String debug_msg;
  // debug_msg.data = "cmd_now_=" + str(joy->buttons.);
  // pub_debug_joy_.publish(debug_msg);
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

