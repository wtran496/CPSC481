#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "turtlesim/Pose.h"
#include <turtlesim/Spawn.h>
#include <turtlesim/Kill.h>
#include <sstream>
#include <string>
#include <sstream>

const int MAX_TTURTLES = 7;
const int MAX_XTURTLES = 10;

using namespace::std;

double getDistance(double x1, double y1, double x2, double y2){
	return sqrt(pow((x1-x2),2) + pow((y1-y2),2));
}

int main(int argc, char **argv){

	ros::init(argc,argv,"hw4");

	int turtle1 = 1; // Store Tturtle1
	int turtle2 = 1; // Store Tturtle2
	double max_distance = 0; // To check the furthest pair
	double distance = 0; // To check pair distances
	stringstream jname; 
	stringstream tname;

	ros::NodeHandle n;
	ros::Rate loop_rate(2);

	//	Wait-for-message time-out duration
	ros::Duration wait_duration = ros::Duration(2);

	//  Defines the message objects
	turtlesim::Pose msg;	
	turtlesim::Pose msg1;
	turtlesim::Pose turtle_1;
	turtlesim::Pose x_turts[MAX_XTURTLES];
	turtlesim::Pose t_turts[MAX_TTURTLES];


	/*
		COLLECTING ALL TURTLE LOCATIONS
	*/
	// Get Turtle1 Information
	string name = "/turtle1/pose";
	try { turtle_1= *(ros::topic::waitForMessage<turtlesim::Pose>(name, wait_duration));}
	catch (int e) { ROS_INFO("COULDNT FIND THAT TURT"); }
	cout<<"Turtle1: "<<turtle_1.x<<", "<<turtle_1.y<<endl;

	//	Collect all the locations of the X-Turtles in x_turtle[]
	for(int x_turt = 0; x_turt < MAX_XTURTLES; x_turt++){
		tname.clear();
		tname.str("");
		tname << "/X" << x_turt+1 << "/pose";
		name = tname.str();

		try { x_turts[x_turt]= *(ros::topic::waitForMessage<turtlesim::Pose>(name, wait_duration));}
		catch (int e) { ROS_INFO("COULDNT FIND THAT TURT"); }
	}
	//	Print all the locations of the X-Turtles to the screen
	for(int i = 0; i<MAX_XTURTLES; i++){
		cout<<"X"<<i+1<<": "<<x_turts[i].x<<", "<<x_turts[i].y<<endl;
	}

	//	Collect all the locations of the T-Turtles in t_turtle[]
	for(int t_turt = 0; t_turt < MAX_TTURTLES; t_turt++){
		tname.clear();
		tname.str("");
		tname << "/T" << t_turt+1 << "/pose";
		name = tname.str();

		try { t_turts[t_turt]= *(ros::topic::waitForMessage<turtlesim::Pose>(name, wait_duration));}
		catch (int e) { ROS_INFO("COULDNT FIND THAT TURT"); }
	}
	//	Print all the locations of the T-Turtles to the screen
	for(int i = 0; i<MAX_TTURTLES; i++){
		cout<<"T"<<i+1<<": "<<t_turts[i].x<<", "<<t_turts[i].y<<endl;
	}



	/*
			GETTING FURTHEST PAIR OF T-TURTLES
	*/
	for(int src_turt = 0; src_turt  < MAX_TTURTLES; src_turt ++ ){
		cout<<"Outer loop:  Turtle--"<<src_turt+1<<endl;	// Added this for personal visualization purposes-- Delete if you want
		float src_x = t_turts[src_turt].x;
		float src_y = t_turts[src_turt].y;

		// src_turt+1 to compare each turtle
		for (int dest_turt = src_turt+1; dest_turt < MAX_TTURTLES; dest_turt++){
			cout<<"Inner loop:  Turtle--"<<dest_turt+1<<endl;	// Added this for personal visualization purposes-- Delete if you want
			float dest_x = t_turts[dest_turt].x;
			float dest_y = t_turts[dest_turt].y;

			// Get the distance
			double distance = getDistance(src_x, src_y, dest_x, dest_y);

			// Check the distance
			ROS_INFO("Distance: %f",distance);
			// Set if this distance is greater
			if (distance > max_distance){
				//Check the pair
				ROS_INFO("PAIR AND DISTANCE: %f , %f", max_distance, distance);
				turtle1 = src_turt;
				turtle2 = dest_turt;
				max_distance = distance;
			}
		}
	}

	//This whole section display the pairs
	cout << "The furthest pairs are " << turtle1+1 << " at " 
	<< t_turts[turtle1].x << "," << t_turts[turtle1].y 
	<< " and " 
	<< turtle2+1 << " at " << t_turts[turtle2].x << "," << t_turts[turtle2].y<<endl;
   return 0;
}