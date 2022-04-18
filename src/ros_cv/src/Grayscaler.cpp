#include "sensor_msgs/image_encodings.h"
#include <Grayscaler.hpp>

ImageConverter::ImageConverter() : it_(nh_) {
  // Subscrive to input video feed and publish output video feed
  image_sub_ =
      it_.subscribe("/ros_cv/greyscale_from", 1, &ImageConverter::imageCb, this);
  image_pub_ = it_.advertise("/ros_cv/greyscale_to", 1);

  cv::namedWindow(OPENCV_WINDOW);
}

ImageConverter::~ImageConverter() { cv::destroyWindow(OPENCV_WINDOW); }

void ImageConverter::imageCb(const sensor_msgs::ImageConstPtr &msg) {
  cv_bridge::CvImagePtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
  } catch (cv_bridge::Exception &e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  // Draw an example circle on the video stream
  // if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
  //   cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255, 0, 0));

  // Update GUI Window
  cv::imshow(OPENCV_WINDOW, cv_ptr->image);
  cv::waitKey(3);

  // Output modified video stream
  image_pub_.publish(cv_ptr->toImageMsg());
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "grayscaler");
  ImageConverter ic;
  ros::spin();
  return 0;
}
