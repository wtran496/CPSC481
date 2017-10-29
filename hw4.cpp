#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "turtlesim/Pose.h"
#include <turtlesim/Spawn.h>
#include <turtlesim/Kill.h>
#include <sstream>
#include <string>
#include <sstream>

using namespace::std;

double getDistance(double x1, double y1, double x2, double y2){
	return sqrt(pow((x1-x2),2) + pow((y1-y2),2));
}

int main(int argc, char **argv){

	ros::init(argc,argv,"hw4");

	int num_turts = 3;
	int turtle1 = 1; // Store Tturtle1
	int turtle2 = 1; // Store Tturtle2
	double pair = 0; // To check the furthest pair
	double distance = 0; // To check pair distances
	stringstream jname; 
	stringstream tname;

	ros::NodeHandle n;
	ros::Rate loop_rate(2);
	ros::Duration wait_duration = ros::Duration(2);

	//  Defines the message objects
	turtlesim::Pose msg;	
	turtlesim::Pose msg1;

	for(int src_turt = 0; src_turt  < num_turts; src_turt ++ ){
		tname.clear();
		tname.str("");
		tname << "/T" << src_turt+1 << "/pose";
		cout<<"Outer loop"<<tname.str()<<endl;	// Added this for personal visualization purposes-- Delete if you want
		string name = tname.str();

		// Get info about src_turt
		try {msg = *(ros::topic::waitForMessage<turtlesim::Pose>(name, ros::Duration(2)));}
		catch (int e) {ROS_INFO("COULDNT FIND THAT TURT");}
		ROS_INFO("X: %f,  Y: %f", msg.x, msg.y);

		// src_turt+1 to compare each turtle
		for (int dest_turt = src_turt+1; dest_turt < num_turts; dest_turt++){
			jname.clear();
			jname.str("");
			jname << "/T" << dest_turt+1 << "/pose";
			cout<<"Inner loop"<<jname.str()<<endl; // Added this for personal visualization purposes-- Delete if you want
			string name = jname.str();

			// Get info about dest_turt
			try{msg1 = *(ros::topic::waitForMessage<turtlesim::Pose>(name, ros::Duration(2)));}
			catch (int e){ROS_INFO("COULDNT FIND THAT TURT");}
			ROS_INFO("X1: %f,  Y1: %f", msg1.x, msg1.y);

			// Get the distance
			double distance = getDistance(msg.x,msg.y,msg1.x,msg1.y);

			// Check the distance
			ROS_INFO("Distance: %f",distance);
			// Set if this distance is greater
			if (distance > pair){
				//Check the pair
				ROS_INFO("PAIR AND DISTANCE: %f , %f",pair,distance);
				turtle1 = src_turt+1;
				turtle2 = dest_turt+1;
				pair = distance;
			}
		}
	}

	//This whole section display the pairs
	tname.clear();
	tname.str("");
	tname << "/T" << turtle1 << "/pose";
	string name = tname.str();
	try {msg = *(ros::topic::waitForMessage<turtlesim::Pose>(name, ros::Duration(2)));}
	catch (int e) {ROS_INFO("COULDNT FIND THAT TURT");}
	name.clear();
		
	jname.clear();
	jname.str("");
	jname << "/T" << turtle2 << "/pose";		
	name = jname.str();
	try {msg1 = *(ros::topic::waitForMessage<turtlesim::Pose>(name, ros::Duration(2)));}
	catch (int e) {ROS_INFO("COULDNT FIND THAT TURT");}
	name.clear();

	cout << "The furthest pairs are " << turtle1 << " at " << msg.x << "," << msg.y << " and " << turtle2 << " at " << msg1.x << "," << msg1.y<<endl;
   return 0;
}
