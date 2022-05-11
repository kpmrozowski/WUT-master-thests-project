#include "ros/console.h"
#include <ros/ros.h>
// #include <darknet_ros_msgs/BoundingBoxes.h>
#include <sensor_msgs/LaserScan.h>
#include <gnc_functions.hpp>
#include <cmath>
#include <fmt/core.h>

template <typename T, typename A>
size_t arg_min(std::vector<T, A> const& vec) {
  return static_cast<size_t>(std::distance(vec.begin(), std::min_element(vec.begin(), vec.end())));
}

void scan_cb(const sensor_msgs::LaserScan::ConstPtr& msg)
{
	double desired_yaw = M_PI / 2.0;
   double desired_distance = 2.0;
   double r_max = 2.0;
	sensor_msgs::LaserScan current_2D_scan;
  	current_2D_scan = *msg;

   size_t idx_front = current_2D_scan.ranges.size() / 4;
   double front_distance = current_2D_scan.ranges[idx_front];
   double x_front = cos(current_2D_scan.angle_increment * idx_front); // should be always 0
   double y_front = sin(current_2D_scan.angle_increment * idx_front); // optimizing for desred_distance
	
   // calculate surrounding points mass center relative pose
   //    it's more robust than going to the closest point :D
   double x_mc = 0.0, y_mc = 0.0, yaw_mc = 0.0, r_mc = 0.0;
   for(int i=1; i<current_2D_scan.ranges.size(); i++)
   {
      if (current_2D_scan.ranges[i] > current_2D_scan.range_max or current_2D_scan.ranges[i] < 1.0) {
         continue;
      }
		float k = 5.0;
      float x = cos(current_2D_scan.angle_increment*i);
      float y = sin(current_2D_scan.angle_increment*i);
      float U = .5*k*pow(1.0/current_2D_scan.ranges[i], 2);	
      x_mc += x*U;
      y_mc += y*U;
   }
   yaw_mc = std::atan2(y_mc, x_mc);
   yaw_mc = (yaw_mc > 0) ? yaw_mc : (2.0 * M_PI + yaw_mc);
   double yaw_mc_idx_low = std::floor(yaw_mc / current_2D_scan.angle_increment);
   double yaw_mc_idx_high = std::ceil(yaw_mc / current_2D_scan.angle_increment);
   r_mc = (current_2D_scan.ranges[yaw_mc_idx_low] + current_2D_scan.ranges[yaw_mc_idx_high]) / 2.0;
   if (r_mc > current_2D_scan.range_max or r_mc < 1.0) {
      if (r_mc < 1.0) {
         ROS_WARN("%s", fmt::format("WARING: r_mc={}", r_mc).c_str());
      }
      return;
   }
   // // r_mc = std::pow(std::pow(x_mc, 2.0) + std::pow(y_mc, 2.0), 0.5);
   // size_t idx_closest = arg_min(current_2D_scan.ranges);
   // double yaw_closest = idx_closest * current_2D_scan.angle_increment;
   // double r_closest = current_2D_scan.ranges[idx_closest];
   // if (r_closest > current_2D_scan.range_max or r_closest < 1.0) {
   //    return;
   // }
   // r_mc = r_closest;
   // yaw_mc = yaw_closest;

   geometry_msgs::Point current_pos = iq::get_current_location();
	float current_heading = iq::get_current_heading();
	float deg2rad = (M_PI/180);

   // compute errors
   double yaw_err = yaw_mc - desired_yaw;
   double r_err = r_mc - desired_distance;

   // compute relative displacement vector perpendicular to the wall in drone CS | OKOK
   double dx_per = r_err * cos(yaw_mc + current_heading*deg2rad);
   double dy_per = r_err * sin(yaw_mc + current_heading*deg2rad);

   // compute relative displacement vector parallel to the wall in drone CS
   double dx_par = ((r_max - r_err > 0) ? r_max - r_err : 0.0)          * sin(yaw_mc + current_heading*deg2rad);
   double dy_par = ((r_max - r_err > 0) ? r_max - r_err : 0.0) * (-1.0) * cos(yaw_mc + current_heading*deg2rad);

   // compute resultant displacement vector in drone CS
   double dx = dx_per + dx_par;
   double dy = dy_per + dy_par;

   // compute resultant relative displacement vector in global CS
   double dx_gcs =   dx * sin(current_heading*deg2rad) + dy * cos(current_heading*deg2rad);
   double dy_gcs = - dx * cos(current_heading*deg2rad) + dy * sin(current_heading*deg2rad);

   ROS_INFO("%s", fmt::format("\n"
      "dx_gcs={}\n"
      "dy_gcs={}\n"
      "yaw_err={}\n"
      "current_heading={}\n"
      "yaw_mc={}\n"
      "r_err={}\n"
      "desired_distance={}\n"
      "r_mc={}\n"
      "dx_per={}\n"
      "dy_per={}\n"
      "dx_par={}\n"
      "dy_par={}\n", dx_gcs, dy_gcs, yaw_err, current_heading*deg2rad, yaw_mc, r_err, desired_distance, r_mc, dx_per, dy_per, dx_par, dy_par).c_str());
   
   // put computed values into destimation pose
   float x = current_pos.x + dx * 100.;
   float y = current_pos.y + dy * 100.;
   float z = 2.0;
   float psi = current_heading + yaw_err / deg2rad;
   if (psi > 360.) {
      psi -= 360.;
   } else if (psi < 0.) {
      psi += 360.;
   }
   iq::set_destination(x, y, z, psi);
   // iq::set_destination(0., 0., z, psi);
   
   // // compute pose of wall located in neighborhood mass center
   // //    in globall coordinate ststem relative to the drone
   // double yaw_mc_gcs = yaw_err + current_heading*deg2rad +  M_PI / 2.0;
   // double dx_mc_gcs = r_mc * sin(yaw_mc_gcs);
   // double dy_mc_gcs = r_mc * cos(yaw_mc_gcs);
   
   // // compute relative displacement vector parallel the wall in drone CS
   // double dx_per_gcs = r_err * sin(yaw_mc_gcs);
   // double dy_per_gcs = r_err * cos(yaw_mc_gcs);

   // // compute relative drift to the right in GCS
   // double dx_par_gcs =   dx_par * sin(current_heading*deg2rad);
   // double dy_par_gcs = - dx_par * cos(current_heading*deg2rad);

   // // compute resultant displacement vector
   // double d
   
	
}

int main(int argc, char **argv)
{
	//initialize ros 
	ros::init(argc, argv, "wall_following_node");
	ros::NodeHandle n;
	// ros::Subscriber sub = n.subscribe("/darknet_ros/bounding_boxes", 1, scan_cb);
	//initialize control publisher/subscribers
	iq::init_publisher_subscriber(n);

  	// wait for FCU connection
	iq::wait4connect();

	//wait for used to switch to mode GUIDED
	iq::wait4start();

	//create local reference frame 
	iq::initialize_local_frame();

	//request takeoff
	iq::takeoff(2);


	iq::set_destination(0,0,2,0);
   // ros::Rate{10}.sleep();
	//specify control loop rate. We recommend a low frequency to not over load the FCU with messages. Too many messages will cause the drone to be sluggish
	ros::Rate rate(0.8);
	int counter = 0;
	ros::Subscriber wall_following_sub = n.subscribe<sensor_msgs::LaserScan>("/spur/laser/scan", 0, scan_cb);
	while(ros::ok())
	{
		
		ros::spinOnce();
		// rate.sleep();
		
	
	
	}

	return 0;
}

