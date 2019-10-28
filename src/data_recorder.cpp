/**
 * \file data_recorder.cpp
 *
 * \author kristyna kumpanova
 *
 * Contact: kumpakri@gmail.com
 */

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <time.h>
#include "sensor_msgs/Image.h"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <boost/format.hpp>

/* creates pointer to handle without initializing it */
boost::shared_ptr<ros::NodeHandle> nh;
boost::shared_ptr<tf::TransformListener> tf_listener;
FILE *f;
time_t zero_time;
time_t elapsed_time;
char* path;

/**
*	Saves image from camera labeled by time since the node was started. 
*/
void saveImageCallback(const sensor_msgs::Image::ConstPtr& msg)
{
	elapsed_time = time(NULL) - zero_time;

	cv_bridge::CvImageConstPtr image;
	//transforms image from msg into OpenCV image
	image = cv_bridge::toCvShare(msg);
	char image_path[100];
	snprintf(image_path, sizeof image_path, "%s/images/image_%d.png", path,elapsed_time);
	std::string s2 = str( boost::format("%s/images/image_%d.png") % path % elapsed_time );

	cv::imwrite(s2, image->image );

}

/**
*	Writes position in the map to a file with time since the node was started. 
*/
void savePositionCallback(const nav_msgs::OccupancyGrid::ConstPtr &map) {
	elapsed_time = time(NULL) - zero_time;

	if (nh->ok()){
		tf::StampedTransform transform;
		try{
			/* get transformation from base_link coordinates to map coordinates */
			tf_listener->lookupTransform("map","base_link", ros::Time(0), transform);

			double pos_x = transform.getOrigin().x();
			double pos_y = transform.getOrigin().y();

			tf::Quaternion q(
				transform.getRotation().x(),
				transform.getRotation().y(),
				transform.getRotation().z(),
				transform.getRotation().w());

			tf::Matrix3x3 m(q);
			double roll, pitch, yaw;
			m.getRPY(roll, pitch, yaw);

			char positions_path[100];
			snprintf(positions_path, sizeof positions_path, "%s/positions.txt", path);
			f = fopen(positions_path, "a");
			if (f == NULL)
			{
				printf("Error opening file!\n");
				exit(1);
			}

			fprintf(f, "%d %f %f %f\n", elapsed_time, pos_x, pos_y, yaw);
			fclose(f);
		} catch (tf::TransformException ex){
			ROS_ERROR("%s",ex.what());
			ros::Duration(1.0).sleep();
		} 
	}
}

/**
*	Writes position in the map to a file with time since the node was started. 
*/
void saveOdomCallback(const nav_msgs::Odometry::ConstPtr &msg) {
	elapsed_time = time(NULL) - zero_time;

	if (nh->ok()){
		
		double pos_x = msg->pose.pose.position.x;
		double pos_y = msg->pose.pose.position.y;

		tf::Quaternion q(
			msg->pose.pose.orientation.x,
			msg->pose.pose.orientation.y,
			msg->pose.pose.orientation.z,
			msg->pose.pose.orientation.w);

		tf::Matrix3x3 m(q);
		double roll, pitch, yaw;
		m.getRPY(roll, pitch, yaw);

		char odom_path[100];
		snprintf(odom_path, sizeof odom_path, "%s/odom.txt", path);
		f = fopen(odom_path, "a");
		if (f == NULL)
		{
			printf("Error opening file!\n");
			exit(1);
		}

		fprintf(f, "%d %f %f %f\n", elapsed_time, pos_x, pos_y, yaw);
		fclose(f);
	} 
	
}

/**
* Extracts data from .bag file
*/
int main(int argc, char **argv)
{
	zero_time = time(NULL);
	ros::init(argc, argv, "data_recorder");

	/* initializes shared ptr */
	nh.reset(new ros::NodeHandle());
	tf_listener.reset(new tf::TransformListener());
	path = argv[1];

	ros::Subscriber map_sub = nh->subscribe("/map", 100, savePositionCallback);
	ros::Subscriber cam_sub = nh->subscribe("frontCamera/image", 10, saveImageCallback);
	ros::Subscriber odom_sub = nh->subscribe("/odom", 1, saveOdomCallback);
	ros::Rate rate(1.0);
	ros::spin();

	return 0;
}
