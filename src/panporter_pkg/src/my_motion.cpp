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
#include <std_srvs/Empty.h>
#include <cmath>
#include <stdio.h>
#include <map>
#include <string>
#include <gazebo_plugins/gazebo_ros_bumper.h>
#include <control_toolbox/pid.h>
// #include "pid.h"

template <typename T> T sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

# define str(x) std::to_string(x)

class PanporterMotion {
public:
  PanporterMotion(const KDL::Tree& tree, const double publish_frequency);
private:
  void reset_sim();
  void callbackBumperState(const gazebo_msgs::ContactsStateConstPtr& state);
  void callbackJointState(const sensor_msgs::JointStateConstPtr& state);
  void imuCallback(const sensor_msgs::Imu::ConstPtr& msg);
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
  void velCallback(const geometry_msgs::Twist::ConstPtr& vel);
  ros::Rate publish_rate_;
  robot_state_publisher::RobotStatePublisher state_publisher_;
  const double robot_width_ = 0.4;
  const double wheel_radious_ = 0.35;
  ros::NodeHandle nh_, nh_private_;
  ros::Publisher pub_wheel_left_, pub_wheel_right_, pub_prism_, pub_debug_joy_, pub_debug_imu_, pub_debug_js_, pub_debug_bumper_;
  ros::Subscriber joy_sub_, imu_sub_, joint_state_sub_, bumper_sub_;
	// float vel_angular_z_;
	float vel_linear_x_;
  float max_linear_vel_ = 5;
  float max_angular_vel_ = 2 * M_PI;
  float max_m_vel_ = 4.;
  std::size_t axis_count_;
  control_toolbox::Pid pid_mp_, pid_ma_, pid_lr_;
  bool pid_antiwindup_ = false;
  double angle_error_, position_error_;
  ros::Time cmd_last_time_, reset_time_, tnow_;
  double pid_cmd_mp_= 0.0, pid_cmd_ma_= 0.0, pid_cmd_lr_ = 0.0;
  double joy_cmd_m_= 0.0, joy_cmd_l_ = 0.0, joy_cmd_r_ = 0.0;
  double sim_cmd_lr_ = 0.0;
  double m_angle_ = 0.0, m_desired_angle_ = 0.0;
  double m_position_ = 0.0, m_desired_position_ = 0.0;
  double l_state_ = 0.0, r_state_ = 0.0;
  int counter1_ = 0;
  int counter1_limit_ = 10000;
  double pid_mp_p_= 1e0, pid_mp_i_= 1*1e-2, pid_mp_d_= 1e0, pid_mp_imax_= 1e5, pid_mp_imin_= -1e5;
  double pid_ma_p_= 1e0, pid_ma_i_= 1*1e-2, pid_ma_d_= 1e0, pid_ma_imax_= 1e5, pid_ma_imin_= -1e5;
  double pid_lr_p_= 1e0, pid_lr_i_= 1*1e-2, pid_lr_d_= 1e0, pid_lr_imax_= 1e5, pid_lr_imin_= -1e5;
  std_srvs::Empty resetWorldSrv_;
  bool reset_state_ = false;
  double max_vel_ = 10.;
  double max_step_time_ = 1.;
  double after_reset_delay_ = 0.5;
  double after_episode_delay_ = 0.5;
  std::size_t reset_counter_ = 0;
  bool debug = false;
};

PanporterMotion::PanporterMotion(const KDL::Tree& tree, const double publish_frequency) : state_publisher_(tree), publish_rate_(publish_frequency)
{
  bumper_sub_ = nh_.subscribe<gazebo_msgs::ContactsState>("/panporter1/state_bumper", 1, &PanporterMotion::callbackBumperState, this);
  joint_state_sub_ = nh_.subscribe<sensor_msgs::JointState>("/panporter1/joint_states", 1, &PanporterMotion::callbackJointState, this);
	imu_sub_ = nh_.subscribe<sensor_msgs::Imu>("/panporter1/imu", 100, &PanporterMotion::imuCallback, this);
	joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 100, &PanporterMotion::joyCallback, this);
  pub_wheel_left_  = nh_.advertise<std_msgs::Float64>("/panporter1/wheel_left_joint_velocity_controller/command", 100);
  pub_wheel_right_ = nh_.advertise<std_msgs::Float64>("/panporter1/wheel_right_joint_velocity_controller/command", 100);
  pub_prism_  = nh_.advertise<std_msgs::Float64>("/panporter1/prism_joint_position_controller/command", 100);
  if (debug) {
    pub_debug_bumper_ = nh_.advertise<std_msgs::String>("/panporter1/debug_bumper", 100);
    pub_debug_joy_ = nh_.advertise<std_msgs::String>("/panporter1/debug_joy", 100);
    pub_debug_imu_ = nh_.advertise<std_msgs::String>("/panporter1/debug_imu", 100);
    pub_debug_js_ = nh_.advertise<std_msgs::String>("/panporter1/debug_js", 100);
  }
  double publish_freq;
  if (nh_private_.param("publish_frequency", publish_freq, publish_frequency)) return;
  publish_rate_ = ros::Rate(publish_freq);
  reset_time_ = ros::Time::now();
  tnow_ = ros::Time::now();
  pid_mp_ = control_toolbox::Pid(pid_mp_p_, pid_mp_i_, pid_mp_d_, pid_mp_imax_, pid_mp_imin_);
  pid_ma_ = control_toolbox::Pid(pid_ma_p_, pid_ma_i_, pid_ma_d_, pid_ma_imax_, pid_ma_imin_);
  pid_lr_ = control_toolbox::Pid(pid_lr_p_, pid_lr_i_, pid_lr_d_, pid_lr_imax_, pid_lr_imin_);
}

void PanporterMotion::reset_sim() {
  reset_state_ = ros::service::call("/gazebo/reset_simulation", resetWorldSrv_);
  publish_rate_.sleep();
  reset_time_ = ros::Time::now();
  pid_mp_.reset();
  pid_ma_.reset();
  pid_lr_.reset();
  ++reset_counter_;
}

void PanporterMotion::callbackBumperState(const gazebo_msgs::ContactsStateConstPtr& state) {
  tnow_ = ros::Time::now();
  if (state->states.size() > 0) {
    reset_sim();
  }
  // if (m_angle_ < 0.02 && m_angle_ > -0.02 && ) (m_angle_ < M_PI / 2) {
  // if (abs(joy_cmd_m_) < 0.01) joy_cmd_m_ = - 10. * m_position_;

  // I1 - integral max, I2 - integrtal min
  // https://docs.ros.org/en/diamondback/api/control_toolbox/html/classcontrol__toolbox_1_1Pid.html
  // https://github.com/awesomebytes/control_toolbox_pid_tutorial
  angle_error_ = m_angle_ - m_desired_angle_;
  position_error_ = m_position_ - m_desired_position_;
  ros::Duration dt = tnow_ - cmd_last_time_;
  cmd_last_time_ = tnow_;
  pid_cmd_mp_ = .1 * pid_mp_.computeCommand(position_error_, dt);
  pid_cmd_ma_ = .1 * pid_ma_.computeCommand(angle_error_, dt);
  pid_cmd_lr_ = .1 * pid_lr_.computeCommand(angle_error_, dt);
  // pid_cmd_m_ = sgn(pid_cmd_m_) * pow(abs(pid_cmd_m_), 0.25);
  // pid_cmd_lr_ = sgn(pid_cmd_m_) * pow(abs(pid_cmd_m_), 0.25);
  
  // sim_cmd_lr_ = 10. * pow( sin( ( (tnow_ - reset_time_).toSec() - 0.2 ) / 5. * M_PI), 2.); // 5 sec
  if ( (tnow_ - reset_time_).toSec() > after_reset_delay_){
    if (  (tnow_ - reset_time_).toSec() < max_step_time_ + after_reset_delay_ )
      sim_cmd_lr_ = max_vel_ * 0.5 * ( 1 + sin( ( (tnow_ - reset_time_).toSec() - 0.5 * max_step_time_ - after_reset_delay_ ) / max_step_time_ * M_PI) ); // 2 sec to max_vel_ m/s
    else if ( (tnow_ - reset_time_).toSec() < 2.0 * max_step_time_ + after_reset_delay_ )
      sim_cmd_lr_ = max_vel_; // 2 sec of max_vel_ m/s
    else if ( (tnow_ - reset_time_).toSec() < 3.0 * max_step_time_ + after_reset_delay_ )
      sim_cmd_lr_ = max_vel_ * 0.5 * ( 1 + sin( ( (tnow_ - reset_time_).toSec() - 1.5 * max_step_time_ - after_reset_delay_ ) / max_step_time_ * M_PI)); // 2 sec to 0 m/s
    else if ( (tnow_ - reset_time_).toSec() < 4.0 * max_step_time_ + after_reset_delay_ )
      sim_cmd_lr_ = 0.0; // 2 sec of 0 m/s
    else if ( (tnow_ - reset_time_).toSec() < 5.0 * max_step_time_ + after_reset_delay_ )
      sim_cmd_lr_ = -max_vel_ * 0.5 * ( 1 + sin( ( (tnow_ - reset_time_).toSec() - 4.5 * max_step_time_ - after_reset_delay_ ) / max_step_time_ * M_PI)); // 2 sec to -max_vel_ m/s
    else if ( (tnow_ - reset_time_).toSec() < 6.0 * max_step_time_ + after_reset_delay_ )
      sim_cmd_lr_ = -max_vel_; // 2 sec of -max_vel_ m/s
    else if ( (tnow_ - reset_time_).toSec() < 7.0 * max_step_time_ + after_reset_delay_ )
      sim_cmd_lr_ = -max_vel_ * 0.5 * ( 1 + sin( ( (tnow_ - reset_time_).toSec() - 5.5 * max_step_time_ - after_reset_delay_ ) / max_step_time_ * M_PI)); // 2 sec to 0 m/s
    else if ( (tnow_ - reset_time_).toSec() < 7.0 * max_step_time_ + after_reset_delay_ + after_episode_delay_ )
      sim_cmd_lr_ = 0.0; // 2 sec of 0 m/s
    else
      reset_sim();
  }

  if (ros::Time::now() - reset_time_ > ros::Duration(after_reset_delay_)) {
    reset_state_ = false;
  }
  if (reset_state_) {
    pid_cmd_lr_ = 0.0;
    pid_cmd_mp_ = 0.0;
    pid_cmd_ma_ = 0.0;
    sim_cmd_lr_ = 0.0;
    m_angle_ = 0.0;
  }
  std_msgs::Float64 wheel_left_vel_msg;
  std_msgs::Float64 wheel_right_vel_msg;
  std_msgs::Float64 prism_pos_msg;
  wheel_left_vel_msg.data = sim_cmd_lr_ + joy_cmd_l_ + pid_cmd_lr_;
  wheel_right_vel_msg.data = sim_cmd_lr_ + joy_cmd_r_ + pid_cmd_lr_;
  prism_pos_msg.data = joy_cmd_m_ + pid_cmd_mp_ + pid_cmd_ma_;
  // wheel_left_vel_msg.data = sim_cmd_lr_;
  // wheel_right_vel_msg.data = sim_cmd_lr_;
  // prism_pos_msg.data = 0.0;
  
  // if (counter1_ > counter1_limit_) {
  //   counter1_=0;
  //   joy_cmd_l_ = 0.0; joy_cmd_r_ = 0.0;
  // }
  // ++counter1_;
  
  pub_wheel_left_.publish(wheel_left_vel_msg);
  pub_wheel_right_.publish(wheel_right_vel_msg);
  pub_prism_.publish(prism_pos_msg);
  if (debug) {
    std_msgs::String debug_msg;
    debug_msg.data = "sim_cmd_lr_=" + str(sim_cmd_lr_) + "   dt_reset=" + str( (tnow_ - reset_time_).toSec() );
    pub_debug_bumper_.publish(debug_msg);
  }
  publish_rate_.sleep();
}

void PanporterMotion::callbackJointState(const sensor_msgs::JointStateConstPtr& state) {
  if (state->name.size() != state->position.size()){
    ROS_ERROR("Robot state publisher received an invalid joint state vector");
    return;
  }
  
  // get joint positions from state message
  // std::map<std::string, double> joint_positions;
  // for (unsigned int i = 0; i < state->name.size(); ++i)
  //   joint_positions.insert(make_pair(state->name[i], state->position[i]));
  // state_publisher_.publishTransforms(joint_positions, state->header.stamp, "panporter1-");
  m_position_ = state->position[0];
  l_state_ = state->position[1];
  r_state_ = state->position[2];
  
  if (debug) {
    std_msgs::String debug_msg;
    debug_msg.data = "";
    for (unsigned int i = 0; i < state->name.size(); ++i)
      debug_msg.data += state->name[i] + "=" + str(state->position[i]) + ", ";
    pub_debug_js_.publish(debug_msg);
  }
}

void PanporterMotion::imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
  if (debug) {
    ROS_INFO("Imu Orientation x: [%f], y: [%f], z: [%f], w: [%f]", 
      msg->orientation.x,
      msg->orientation.y,
      msg->orientation.z,
      msg->orientation.w
    );
  }
  m_angle_ = msg->orientation.y;
  // if (m_angle_ > 0.12 || m_angle_ < -0.12) m_angle_ = m_desired_angle_;
  
  if (debug) {
    double pe_mp,  ie_mp,  de_mp;
    double pe_ma,  ie_ma,  de_ma;
    double pe_lr, ie_lr, de_lr;
    pid_mp_.getCurrentPIDErrors(&pe_mp, &ie_mp, &de_mp);
    pid_ma_.getCurrentPIDErrors(&pe_ma, &ie_ma, &de_ma);
    pid_lr_.getCurrentPIDErrors(&pe_lr, &ie_lr, &de_lr);
    std_msgs::String debug_msg;
    debug_msg.data = "pid_cmd_mp_=" + str(pid_cmd_mp_) + ", pe_mp=" + str(pid_mp_p_ * pe_mp) + ", ie_mp=" + str(pid_mp_i_ * ie_mp) + ", de_mp=" + str(pid_mp_d_ * de_mp) + ",              "
                   + "pid_cmd_ma_=" + str(pid_cmd_ma_) + ", pe_ma=" + str(pid_ma_p_ * pe_ma) + ", ie_ma=" + str(pid_ma_i_ * ie_ma) + ", de_ma=" + str(pid_ma_d_ * de_ma) + ",      "
                   + "pid_cmd_lr_=" + str(pid_cmd_lr_) + ", pe_lr=" + str(pid_lr_p_ * pe_lr) + ", ie_lr=" + str(pid_lr_i_ * ie_lr) + ", de_lr=" + str(pid_lr_d_ * de_lr)
                   + ",            m_angle_=" + str(m_angle_);
    pub_debug_imu_.publish(debug_msg);
  }
}

void PanporterMotion::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
	double vel_angular_z = joy->axes[0];
	vel_linear_x_ = joy->axes[1];
  joy_cmd_m_ = max_m_vel_ * joy->axes[4];
  if      (joy->buttons[0]) pid_ma_p_ *= 1.5;
  else if (joy->buttons[1]) pid_ma_p_ /= 1.5;
  else if (joy->buttons[2]) pid_ma_i_ *= 1.5;
  else if (joy->buttons[3]) pid_ma_i_ /= 1.5;
  else if (joy->buttons[4]) pid_ma_d_ *= 1.5;
  else if (joy->buttons[5]) pid_ma_d_ /= 1.5;
  else if (joy->buttons[6]) pid_ma_imax_ *= 1.5;
  else if (joy->buttons[7]) pid_ma_imax_ /= 1.5;
  else if (joy->buttons[8]) pid_ma_imin_ *= 1.5;
  else if (joy->buttons[9]) pid_ma_imin_ /= 1.5;
  pid_ma_.setGains(pid_ma_p_, pid_ma_i_, pid_ma_d_, pid_ma_imax_, pid_ma_imin_, pid_antiwindup_);
  
  joy_cmd_l_ = (max_linear_vel_ * vel_linear_x_ - max_angular_vel_ * vel_angular_z * robot_width_ / 2.) / wheel_radious_;
  joy_cmd_r_ = (max_linear_vel_ * vel_linear_x_ + max_angular_vel_ * vel_angular_z * robot_width_ / 2.) / wheel_radious_;
  
  if (debug) {
    std_msgs::String debug_msg;
    // debug_msg.data = "buttons: ";
    // for (int i = 0; i < joy->buttons.size(); ++i)
    //   debug_msg.data += "b" + str(i) + "=" + str(joy->buttons[i]) + ",";
    debug_msg.data = "\np=" + str(pid_ma_p_) + ",i=" + str(pid_ma_i_*1e5) + ",d=" + str(pid_ma_d_*1e-3) + ",Imax=" + str(pid_ma_imax_*1e-5) + ",Imin=" + str(pid_ma_imin_*1e-5);
    pub_debug_joy_.publish(debug_msg);
  }
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
    double freq = 100;
    ros::Rate loop_rate(freq);
    PanporterMotion motion(tree, freq);

    ros::spin();;
  }
  
  // ros::Rate loop_rate(10);
  // loop_rate.sleep();

  return 0;
}

