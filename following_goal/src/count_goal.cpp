/**
*\file count_goal.cpp
*\brief Counting goals for the Gazebo robot
*\author Simone Borelli
*\version 1.0
*\date 03/03/2023
*
*\details
*
* Subscribes to : <BR>
*	/reaching_goal/result
*
*
* Service : <BR>
*	/result
* 
* Custom service : <BR>
*       NumberGoals
*
* Description : 
*
* This is a service node, that manages the information about
* number of goals reached by the robot 
* and cancelled by the user.
* It interacts with  NumberGoals, that is a custom service.
*/




#include "following_goal/NumberGoals.h"
#include "ros/ros.h"
#include "assignment_2_2022/PlanningActionResult.h"

// global variables
int reached=0; ///< A variable for number of reached goal
int eliminated=0; ///< A variable for number of deleted goal
int status; ///< A variable for the status of the goal

/* similar to callback function, set in the custom service:
	number of reach goals;
	number of eliminated goals */

/**
*\brief a service callback
*\param req defines the request to NumberGoals custom service
*\param res defines the reponse of NUmberGoals custom service
*
*\return always true, as a convention of service callback
*
* This callback assigns the values of variables reached and eliminated
* to the response part of custom service
*
*/
bool goal_service (following_goal::NumberGoals::Request &req, following_goal::NumberGoals::Response &res) {
	res.reached=reached;
	res.cancelled=eliminated;
	return true;
}


/* this callback takes the status of current node and counts goals*/

/**
*\brief a subscription callback
*\param msg define the type of message
*
*\return nothing, because it is a void function
*
* This is a callback, that prints the status of the goal, and checks its value :
* if it is 3, increment by one the variable reached, whereas if it is 2, increment
* by one the variable eliminated.
*
*/
void count_reach_canc_Callback (const assignment_2_2022::PlanningActionResult::ConstPtr& msg) {
	
	status=msg->status.status;
	ROS_INFO("Status goal:%d", status);
	
	// check if the current goal is succeeded
	if (status==3) 
		reached++;
	
	// check if the current goal is preemptive
	else if ( status==2) 
		eliminated++;
}

/**
*\brief the main function, that manages everything
*\param argc is the number of arguments, provided by argv
*\param argv containes the arguments, passed by comand line 
*
*\return the number 0 as a convention of any main function in C++
*
* The main function uses the callback count_reach_canc_Callback,
* for subscribing the status of the goal from the topic
* reaching_goal/result, and uses the callback goal_service, for sending
* the information about numbers of reached and deleted goal to /result.
*
*/

int main (int argc, char** argv) {
	
	/* you must call one of the versions of ros::init() before using any other
  	part of the ROS system. */
	ros::init(argc, argv, "count_goal");
	
	// NodeHandle is the main access point to communications with the ROS system.
	ros::NodeHandle n;
	
	// define a subscriber, which takes information from topic reaching_goal/result
	ros::Subscriber sub=n.subscribe("reaching_goal/result", 1, count_reach_canc_Callback);
	
	// define a service, which send information on service result
	ros::ServiceServer service=n.advertiseService("/result", goal_service);
	sleep(1);
	ros::spin();
	return 0;

}
