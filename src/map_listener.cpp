/**
 * \file map_listener.cpp
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

//nav_msgs::OccupancyGrid::ConstPtr map; 
//IDEA: move transform from callback to main
//      use map as global variable for map data
//      callback: map = msg
//      main: test if map is empty / new map data was acquired

/* creates pointer to handle without initializing it */
boost::shared_ptr<ros::NodeHandle> nh;
boost::shared_ptr<tf::TransformListener> tf_listener;

/**
*	Prints position in map into terminal.
*/
void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr &map) 
{
	if (nh->ok())
	{
		tf::StampedTransform transform;
		try {
			/* get transformation from base_link coordinates to map coordinates */
			tf_listener->lookupTransform("map","base_link",ros::Time(0), transform);

			double true_x = transform.getOrigin().x();
			double true_y = transform.getOrigin().y();
			ROS_INFO("True x: [%f], true y: [%f];\n", true_x, true_y);
		} catch (tf::TransformException ex){
			ROS_ERROR("%s",ex.what());
			ros::Duration(1.0).sleep();
		}
	} else {
		ROS_ERROR("map_listener.cpp node handle not ok");
	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "map_listener");

	/* initializes shared ptr */
	nh.reset(new ros::NodeHandle());
	tf_listener.reset(new tf::TransformListener());

	ros::Subscriber map_sub = nh->subscribe("/map", 10, mapCallback);
	ros::Rate rate(10.0);
	ros::spin();

	return 0;
}
