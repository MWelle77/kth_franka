#include "kth_pandabaer/calibration_test.cpp"

#include <ros/ros.h>		
#include <ros/package.h>
#include <iostream>





int main(int argc, char **argv)
{
	ros::init(argc, argv, "calibration_test");
	ros::NodeHandle n;


	ros::spin();

  	return 0;
}