#include<ros/ros.h>
#include<geometry_msgs/Twist.h>
#include<iostream>

class PanporterMotion {
public:
  PanporterMotion();
private:
  void velCallback(const geometry_msgs::Twist::ConstPtr& vel);
  ros::NodeHandle n;
  ros::Publisher pub;
  ros::Subscriber sub;
};

PanporterMotion::PanporterMotion()
{
  sub = n.subscribe("cmd_vel", 10, &PanporterMotion::velCallback, this);
}

void PanporterMotion::velCallback(const geometry_msgs::Twist::ConstPtr& vel)
{
   geometry_msgs::Twist new_vel = *vel;
   if (vel->linear.x > 1.8) {
     new_vel.linear.x = 1.8;
     printf("vel=%f\n", vel->linear.x);
   }
  //  pub.publish(new_vel);
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "my_motion");
  PanporterMotion motion;

  ros::spin();

  return 0;
}

