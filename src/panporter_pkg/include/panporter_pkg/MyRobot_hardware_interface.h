#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <joint_limits_interface/joint_limits_urdf.h>
#include <hardware_interface/robot_hw.h>
#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <controller_manager/controller_manager.h>
#include <boost/scoped_ptr.hpp>
#include <boost/function.hpp>
#include <ros/ros.h>

#include <cassert>
#include <cmath>
#include <sstream>

#include <boost/bind.hpp>
#include <boost/shared_ptr.hpp>
#include <std_msgs/Float32.h>

#include <controller_interface/controller.h>
#include <pluginlib/class_list_macros.h>
#include "controller_manager/controller_spec.h"

#include <cstdio>
#include <map>
#include <mutex>
#include <string>
#include <vector>
#include <ros/node_handle.h>
#include <pluginlib/class_loader.hpp>
#include <controller_manager_msgs/ListControllerTypes.h>
#include <controller_manager_msgs/ListControllers.h>
#include <controller_manager_msgs/ReloadControllerLibraries.h>
#include <controller_manager_msgs/LoadController.h>
#include <controller_manager_msgs/UnloadController.h>
#include <controller_manager_msgs/SwitchController.h>
#include <controller_manager/controller_loader_interface.h>
#include <hardware_interface/internal/hardware_resource_manager.h>



namespace controller_ns{

class MyRobot : public hardware_interface::RobotHW 
{
    public:
        MyRobot(ros::NodeHandle& nh);
        ~MyRobot();
        void init();
        void update(const ros::TimerEvent& e);
        void read();
        void write(ros::Duration elapsed_time);
        
    protected:
        hardware_interface::JointStateInterface joint_state_interface_;
        hardware_interface::EffortJointInterface effort_joint_interface_;
        // hardware_interface::VelocityJointInterface velocity_joint_interface_;
        hardware_interface::PositionJointInterface position_joint_interface_;
        
        joint_limits_interface::JointLimits limits;
        joint_limits_interface::EffortJointSaturationInterface effortJointSaturationInterface;
        // joint_limits_interface::VelocityJointSaturationInterface velocityJointSaturationInterface;
        joint_limits_interface::PositionJointSaturationInterface positionJointSaturationInterface;
        

        double joint_position_[3];
        double joint_velocity_[3];
        double joint_effort_[3];
        double joint_effort_command_[2];
        // double joint_velocity_command_[2];
        double joint_position_command_;
        
        ros::NodeHandle nh_;
        ros::Timer my_control_loop_;
        ros::Duration elapsed_time_;
        double loop_hz_;
        boost::shared_ptr<controller_manager::ControllerManager> controller_manager_;
};

class PositionController : public controller_interface::Controller<hardware_interface::EffortJointInterface>
{
public:
  bool init(hardware_interface::EffortJointInterface* hw, ros::NodeHandle &n)
  {
    // get joint name from the parameter server
    std::string my_joint;
    if (!n.getParam("joint", my_joint)){
      ROS_ERROR("Could not find joint name");
      return false;
    }

    // get the joint object to use in the realtime loop
    joint_ = hw->getHandle(my_joint);  // throws on failure
    return true;
  }

  void update(const ros::Time& time, const ros::Duration& period)
  {
    double error = setpoint_ - joint_.getPosition();
    joint_.setCommand(error*gain_);
  }

  void starting(const ros::Time& time) { }
  void stopping(const ros::Time& time) { }

private:
  hardware_interface::JointHandle joint_;
   static constexpr double gain_ = 1.25;
   static constexpr double setpoint_ = 3.00;
};
PLUGINLIB_EXPORT_CLASS(controller_ns::PositionController, controller_interface::ControllerBase);

} //namespace


// class MyRobot : public hardware_interface::RobotHW
// {
// public:
//   MyRobot() 
//  { 
//    // connect and register the joint state interface
//    hardware_interface::JointStateHandle state_handle_a("A", &pos[0], &vel[0], &eff[0]);
//    jnt_state_interface.registerHandle(state_handle_a);

//    hardware_interface::JointStateHandle state_handle_b("B", &pos[1], &vel[1], &eff[1]);
//    jnt_state_interface.registerHandle(state_handle_b);

//    registerInterface(&jnt_state_interface);

//    // connect and register the joint position interface
//    hardware_interface::JointHandle pos_handle_a(jnt_state_interface.getHandle("A"), &cmd[0]);
//    jnt_pos_interface.registerHandle(pos_handle_a);

//    hardware_interface::JointHandle pos_handle_b(jnt_state_interface.getHandle("B"), &cmd[1]);
//    jnt_pos_interface.registerHandle(pos_handle_b);

//    registerInterface(&jnt_pos_interface);
//   }

// private:
//   hardware_interface::JointStateInterface jnt_state_interface;
//   hardware_interface::PositionJointInterface jnt_pos_interface;
//   double cmd[2];
//   double pos[2];
//   double vel[2];
//   double eff[2];
// };