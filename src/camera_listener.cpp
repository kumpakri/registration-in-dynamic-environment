/**
 * \file camera_listener.cpp
 *
 * \author kristyna kumpanova
 *
 * Contact: kumpakri@gmail.com
 */

#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <boost/format.hpp>

/**
* Saves images from camera into folder.
*/
void cameraCallback(const sensor_msgs::Image::ConstPtr& msg)
{
	// safe image to folder
	ROS_INFO("Seq: [%d]\n", msg->header.seq);
	ros::Time stamp;
	stamp = msg->header.stamp;
	uint32_t secs = stamp.sec; 
	ROS_INFO("Stamp: [%i]\n", secs);

	cv_bridge::CvImageConstPtr image;
	image = cv_bridge::toCvShare(msg);

	/*std::ostringstream stringStream;
	stringStream << "image_" << std::to_string(secs) << ".png";
	std::string copyOfStr = stringStream.str();*/
	std::string s2 = str( boost::format("src/data_acquisition/images/image_%d.png") % msg->header.seq );

	cv::imwrite(s2, image->image );

	//  ROS_INFO("ID: [%s]\n", msg->header.frame_id.c_str());

}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "camera_listener");
	ros::NodeHandle nh;

	ros::Subscriber sub = nh.subscribe("frontCamera/image", 10, cameraCallback);

	ros::spin();

	return 0;
}
