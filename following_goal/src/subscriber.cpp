/**
*\file subscriber.cpp
*\brief Subscribing robot position and velocity
*\author Simone Borelli
*\version 1.0
*\date 03/03/2023
*
*\details
*
* Subscribes to : <BR>
*	/info_robot <BR>
*       /reaching_goal/goal
*
* Publishes to : <BR>
*	[None]
*
*
* Parameters: <BR>
*       frequency
*
* Description : 
*
* This node subscribes position ( x and y) and velocity ( linea vel_x and angular vel_z)
* from the topic /info_robot. Then it subscribes also the goal from 
* the topic /reaching_goal/goal and computes the distance
* from the target and the robot's average speed.
* In addition it uses a parameter to set how
* fast the node publishes the information.
*/

#include "ros/ros.h"
#include <unistd.h>
#include "following_goal/Robot_pv.h"
#include <following_goal/PlanningActionGoal.h>
#include <cmath>


// global variables
float current_posx; ///< A variable that stands for current position of robot along x
float current_posy; ///< A variable that stands for current position of robot along y
float goal_posx; ///< A variable that stands for goal position of robot along x
float goal_posy; ///< A variable that stands for goal position of robot along y
float dist_togx; ///< A variable that stands for distance from the target along x
float dist_togy; ///< A variable that stands for distance from the target along y
float vx; ///< A variable that stands for linear velocity of robot along x
float vz; ///< A variable that stands for angular velocity of robot along z
float aver_speed_x; ///< A variable that stands for the average of linear velocities
float aver_speed_z; ///< A variable that stands for the average of angular velocities
float sum_velx; ///< A variable that stands for the sum of linear velocities
float sum_velz; ///< A variable that stands for the sum of angular velocities
int count_v_lin=1; ///< A variable for the total number of linear speeds
int count_v_ang=1; ///< A variable for the total number of angular speeds

// boolean variable for setting, at the beginning, the two averages
bool initial_linear=true; ///< A variable for setting the initial linear speed
bool initial_angular=true; ///< A variable for setting the initial angular speed
double frequency; ///< A variable for storing the frequency

// callback for subscribing current_position

/**
*\brief a subscriber callback
*\param msg define the type of message
*
*\return nothing, because it is a void function
*
* This callback subscribers position ( x and y) 
* and velocity ( linear vel_x and  angular vel_z)
* and stores them into another variables.
*
*/

void pos_v_Callback(const following_goal::Robot_pv::ConstPtr& msg) {	

	current_posx=msg->posx;
	current_posy=msg->posy;
	vx=msg->velx;
	vz=msg->velz;
}

// callback for subscribing goal position

/**
*\brief a subscriber callback
*\param msg define the type of message
*
*\return nothing, because it is a void function
*
* This callback subscribers the target position ( x and y)
* and stores them into another variables.
*
*/
void pos_v_goal_Callback(const following_goal::PlanningActionGoal::ConstPtr& msg) {	

	goal_posx=msg->goal.target_pose.pose.position.x;
	goal_posy=msg->goal.target_pose.pose.position.y;
}

// this function computes the eucledian distance from the goal to the current position

/**
*\brief function that computes distance from target
*\param [None]
*
*\return nothing, because it is a void function
*
* This function computes the euclidian distance from 
* the target and checks if the goal is reached or not
* for setting the initial linear and angular speeds.
*
*/
void current_goal_dist() {
	
	// component x of distance from goal
	dist_togx=abs (goal_posx-current_posx);
	
	// component y of distance from goal
	dist_togy=abs (goal_posy-current_posy);
	
	// compute euclidian distance
	float eucl_distance=sqrt(dist_togx*dist_togx+dist_togy*dist_togy);
	ROS_INFO("Distance goal pos:%f", eucl_distance);
	
	/* check if the robot has reached the goal, 
		or it stops at initial position*/
	if (eucl_distance<0.55 || eucl_distance==current_posx || eucl_distance==current_posy) {
		initial_linear=true;
		initial_angular=true;
		count_v_lin=1;
		count_v_ang=1;
	}
}

// this function computes the average linear speed

/**
*\brief function that computes average linear speed
*\param [None]
*
*\return nothing, because it is a void function
*
* This function computes the average linear speed
* as the sum of all linear speeds up to now, divided
* by the number of these speeds. At the end
* increments by one the counter of linear speed.
*
*/
void average_speed_lin() {

	/* when everything starts 
		the average speed is equal to current speed*/
	if ( initial_linear) {
		sum_velx=abs(vx);
		initial_linear=false;
	}
	
	// sum all speeds up to now
	else 
		sum_velx=sum_velx+abs(vx);
	
	// compute the average
	aver_speed_x=sum_velx/count_v_lin;
	ROS_INFO("Average linear speed:%f", aver_speed_x);
	
	// increment the number of total speeds
	count_v_lin++;
}

// this function computes the average angular speed

/**
*\brief function that computes average angular speed
*\param [None]
*
*\return nothing, because it is a void function
*
* This function computes the average angular speed
* as the sum of all angular speeds up to now, divided
* by the number of these speeds. At the end
* increments by one the counter of angular speed.
*
*/
void average_speed_ang() {

	/* when everything starts 
		the average speed is equal to current speed*/
	if ( initial_angular) {
		sum_velz=abs(vz);
		initial_angular=false;
	}
	
	// sum all speeds up to now
	else 
		sum_velz=sum_velz+abs(vz);
		
	// compute the average
	aver_speed_z=sum_velz/count_v_ang;
	ROS_INFO("Average angular speed:%f", aver_speed_z);
	
	// increment the number of total speeds
	count_v_ang++;
}

/**
*\brief the main function, that manages everything
*\param argc is the number of arguments, provided by argv
*\param argv containes the arguments, passed by comand line 
*
*\return the number 0 as a convention of any main function in C++
*
* The main function gets the frequency, which is used to set the rate
* of how fast the node publishes the information, as a parameter server.
* It uses the callback pos_v_Callback for subscribing the current position
* x and y from the topic /info_robot, and it uses the callback
* pos_v_goal_Callback for subscribing the position of target. Finally 
* it calls continuosly the functions which computes, respectively,
* the distance from the goal, the average linear and angular speed.
*
*/

int main ( int argc, char ** argv) {
	
	/* you must call one of the versions of ros::init() before using any other
  	part of the ROS system. */
	ros::init(argc, argv, "robot_subscriber");
	
	// NodeHandle is the main access point to communications with the ROS system.
	ros::NodeHandle n;
	
	// get frequency
	ros::param::get("frequency", frequency);
	
	// set the rate
	ros::Rate rate(frequency);
	
	/* this method tells how is the subscriber:
		the first argument tells the topic where it takes the information
		the second one is the size of message queue
		the third is the callback that is called each time
	*/
	ros::Subscriber sub1=n.subscribe("/info_robot", 1, pos_v_Callback);
	
	/* this method tells how is the subscriber:
		the first argument tells the topic where it takes the information
		the second one is the size of message queue
		the third is the callback that is called each time
	*/
	ros::Subscriber sub2=n.subscribe("reaching_goal/goal", 1, pos_v_goal_Callback);
	
	while (ros::ok()) {
		
		// call the function for computing distance
		current_goal_dist();
		
		// call the functions for computing average velocities
		average_speed_lin();
		average_speed_ang();
		rate.sleep();
		ros::spinOnce();
	}
	return 0;
}
