/**
*\file publisher.cpp
*\brief Publishing robot position and velocity
*\author Simone Borelli
*\version 1.0
*\date 03/03/2023
*
*\details
*
* Subscribes to : <BR>
*	/odom
*
* Publishes to : <BR>
*	/info_robot
*
* Custom message : <BR>
*	Robot_pv
*
* Description : 
*
* This node publishes the position and velocity 
* as a custom message, subscribing these values from the topic /odom 
*/

#include "ros/ros.h"
#include <unistd.h>
#include "nav_msgs/Odometry.h"
#include "following_goal/Robot_pv.h"


// global variables
float posx; ///< A variable that stands for position along x
float posy; ///< A variable that stands for position along y
float vx; ///< A variable that stands for linear velocity along x
float vz; ///< A variable that stands for angular velocity along z
ros::Publisher pub; ///< A variable for interact with the publisher 

/**
*\brief a subscriber callback
*\param msg define the type of message
*
*\return nothing, because it is a void function
*
* This callback subscribers position ( x and y) 
* and velocity ( linear vel_x and  angular vel_z)
* and store these variables into 
* those of custom message Robot_pv. Finally it publishes them
*
*/

void pos_v_Callback(const nav_msgs::Odometry::ConstPtr& msg) {	
	
	ROS_INFO("Robot pos and vel:[%f, %f, %f, %f]", msg->pose.pose.position.x, msg->pose.pose.position.y, msg->twist.twist.linear.x, msg->twist.twist.angular.z);
	
	/* subscribe position and 
	velocity from topic odom */
	posx=msg->pose.pose.position.x;
	posy=msg->pose.pose.position.y;
	vx=msg->twist.twist.linear.x;
	vz=msg->twist.twist.angular.z;
	following_goal::Robot_pv pv;
	
	// assign variables to custom message
	pv.posx=posx;
	pv.posy=posy;
	pv.velx=vx;
	pv.velz=vz;
	
	// publish the information
	pub.publish(pv);
}

/**
*\brief the main function, that manages everything
*\param argc is the number of arguments, provided by argv
*\param argv containes the arguments, passed by comand line 
*
*\return the number 0 as a convention of any main function in C++
*
* The main function uses pos_v_Callback for subscribing position and
* velocity of the robot. In addition it publishes these values
* on the topic /info_robot
*
*/

int main(int argc, char ** argv) {

	/* you must call one of the versions of ros::init() before using any other
  	part of the ROS system. */
	ros::init(argc, argv, "robot_publisher");

	// NodeHandle is the main access point to communications with the ROS system.
	ros::NodeHandle n;
	
	/* this method tells how is the subscriber:
		the first argument tells the topic where it takes the information
		the second one is the size of message queue
		the third is the callback that is called each time */
	ros::Subscriber sub=n.subscribe("/odom", 1, pos_v_Callback);
	
	/* this method tells how is the publisher:
		the first argument tells the topic where it publish the information
		the second one is the size of message queue */
	pub=n.advertise<following_goal::Robot_pv>("/info_robot", 1);
	ros::spin();
	return 0;
}
