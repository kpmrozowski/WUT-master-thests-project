#include "../include/panporter_pkg/MyRobot_hardware_interface.h"

// main()
// {
//   MyRobot panporter;
//   controller_manager::ControllerManager cm(&panporter);

//   while (true)
//   {
//      panporter.read();
//      cm.update(panporter.get_time(), panporter.get_period());
//      panporter.write();
//      sleep(1000);
//   }
// }

using namespace controller_ns;
using namespace hardware_interface;
using namespace joint_limits_interface;

MyRobot::MyRobot(ros::NodeHandle& nh) : nh_(nh) {
    
// Declare all JointHandles, JointInterfaces and JointLimitInterfaces of the robot.
    init();
    
// Create the controller manager
    controller_manager_.reset(new controller_manager::ControllerManager(this, nh_));
    
//Set the frequency of the control loop.
    loop_hz_=10;
    ros::Duration update_freq = ros::Duration(1.0/loop_hz_);
    
//Run the control loop
    my_control_loop_ = nh_.createTimer(update_freq, &MyRobot::update, this);
}

MyRobot::~MyRobot() {
}

void MyRobot::init() {
    
// Create joint_state_interface for JointA
    JointStateHandle wheel_right_jointStateHandle("wheel_right_joint", &joint_position_[0], &joint_velocity_[0], &joint_effort_[0]);
    joint_state_interface_.registerHandle(wheel_right_jointStateHandle);
// Create effort joint interface as JointA accepts effort command.
    JointHandle wheel_right_jointEffortHandle(wheel_right_jointStateHandle, &joint_effort_command_[0]);
    effort_joint_interface_.registerHandle(wheel_right_jointEffortHandle); 
// Create Joint Limit interface for JointA
    getJointLimits("wheel_right_joint", nh_, limits);
    EffortJointSaturationHandle wheel_right_jointLimitsHandle(wheel_right_jointEffortHandle, limits);
    effortJointSaturationInterface.registerHandle(wheel_right_jointLimitsHandle);   
    
// Create joint_state_interface for JointB
    JointStateHandle wheel_left_jointStateHandle("wheel_left_joint", &joint_position_[1], &joint_velocity_[1], &joint_effort_[1]);
    joint_state_interface_.registerHandle(wheel_left_jointStateHandle);
// Create effort joint interface as JointB accepts effort command..
    JointHandle wheel_left_jointEffortHandle(wheel_left_jointStateHandle, &joint_effort_command_[1]);
    effort_joint_interface_.registerHandle(wheel_left_jointEffortHandle);
// Create Joint Limit interface for JointB
    getJointLimits("wheel_left_joint", nh_, limits);
    EffortJointSaturationHandle wheel_left_jointLimitsHandle(jointEffortHandleB, limits);
    effortJointSaturationInterface.registerHandle(wheel_left_jointLimitsHandle);
    
// // Create joint_state_interface for JointC
//     JointStateHandle prism_left_jointStateHandle("prism_left_joint", &joint_position_[2], &joint_velocity_[2], &joint_effort_[2]);
//     joint_state_interface_.registerHandle(prism_left_jointStateHandle);
// // Create position joint interface as JointC accepts position command.
//     JointHandle prism_left_jointPositionHandle(prism_left_jointStateHandle, &joint_position_command_);
//     position_joint_interface_.registerHandle(prism_left_jointPositionHandle);
// // Create Joint Limit interface for JointC
//     getJointLimits("prism_left_joint", nh_, limits);
//     PositionJointSaturationHandle prism_left_jointLimitsHandle(prism_left_jointPositionHandle, limits);
//     positionJointSaturationInterface.registerHandle(prism_left_jointLimitsHandle);

// Create joint_state_interface for JointB
    JointStateHandle prism_left_jointStateHandle("prism_left_joint", &joint_position_[1], &joint_velocity_[1], &joint_effort_[1]);
    joint_state_interface_.registerHandle(prism_left_jointStateHandle);
// Create effort joint interface as JointB accepts effort command..
    JointHandle prism_left_jointEffortHandle(prism_left_jointStateHandle, &joint_effort_command_[1]);
    effort_joint_interface_.registerHandle(prism_left_jointEffortHandle);
// Create Joint Limit interface for JointB
    getJointLimits("prism_left_joint", nh_, limits);
    EffortJointSaturationHandle prism_left_jointLimitsHandle(jointEffortHandleB, limits);
    effortJointSaturationInterface.registerHandle(prism_left_jointLimitsHandle);
    
// Register all joints interfaces    
    registerInterface(&joint_state_interface_);
    registerInterface(&effort_joint_interface_);
    registerInterface(&position_joint_interface_);
    registerInterface(&effortJointSaturationInterface);
    registerInterface(&positionJointSaturationInterface);    
}

//This is the control loop
void MyRobot::update(const ros::TimerEvent& e) {
    elapsed_time_ = ros::Duration(e.current_real - e.last_real);
    read();
    controller_manager_->update(ros::Time::now(), elapsed_time_);
    write(elapsed_time_);
}

void MyRobot::read() {
  // Write the protocol (I2C/CAN/ros_serial/ros_industrial)used to get the current joint position and/or velocity and/or effort
  
  //from robot.
  // and fill JointStateHandle variables joint_position_[i], joint_velocity_[i] and joint_effort_[i]
  
}

void MyRobot::write(ros::Duration elapsed_time) {
  // Safety
  effortJointSaturationInterface.enforceLimits(elapsed_time);   // enforce limits for JointA and JointB
  positionJointSaturationInterface.enforceLimits(elapsed_time); // enforce limits for JointC
  
  // Write the protocol (I2C/CAN/ros_serial/ros_industrial)used to send the commands to the robot's actuators.
  // the output commands need to send are joint_effort_command_[0] for JointA, joint_effort_command_[1] for JointB and 
  
  //joint_position_command_ for JointC.
  
}

int main(int argc, char** argv)
{
    
    //Initialze the ROS node.
    ros::init(argc, argv, "MyRobot_hardware_inerface_node");
    ros::NodeHandle nh;
    
    //Separate Sinner thread for the Non-Real time callbacks such as service callbacks to load controllers
    ros::MultiThreadedspinner(2); 
    

    // Create the object of the robot hardware_interface class and spin the thread. 
    MyRobot ROBOT(nh);
	ros::spin();
    return 0;
}
