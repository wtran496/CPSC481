#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "turtlesim/Pose.h"
#include <sstream>
#include <turtlesim/Spawn.h>
#include <turtlesim/Kill.h>
#include <string>
#include <sstream>

using namespace::std;
ros::Publisher velocity_publisher;
ros::Subscriber pose_subscriber;
turtlesim::Pose turtlesim_pose;
const double PI = 3.14159265359;
const int MAX_TTURTLES = 7;
const int MAX_XTURTLES = 10;

void poseCallback(const turtlesim::Pose::ConstPtr &pose_message){
	turtlesim_pose.x = pose_message -> x;
	turtlesim_pose.y = pose_message -> y;
	turtlesim_pose.theta = pose_message -> theta;
}
double degrees2radians(double angle_in_degrees){
	return angle_in_degrees * PI / 180.0;
}
double getDistance(double x1, double y1, double x2, double y2){
return sqrt(pow((x1-x2),2) + pow((y1-y2),2));
}

//		turtle1			turtle2
void moveGoal (double x, double y, double x1, double y1) {
	geometry_msgs::Twist vel_msg;
	ros::Rate loop_rate(100);
	double E = 0.0;
//t checks the movement our turtle goes from either turtle1 or turtle2
	double t1 = x;
	double t2 = y;
	do {
	/****** Proportional Controller ******/
	//linear velocity in the x-axis
	double Kv = 1.5;
	//double Ki = 0.02;
	//double v0 = 2.0;
	//double alpha = 0.5;

	//getDistance calculates Euclidean distance
	double e = getDistance(turtlesim_pose.x, turtlesim_pose.y, t1, t2);
	
	//double E = E + e;
	//Kv = v0 * (exp(-alpha)*error*error)/(error*error); //try something else
	vel_msg.linear.x = (Kv*e);
	vel_msg.linear.y = 0;
	vel_msg.linear.z = 0;
	//angular velocity in the z-axis
	vel_msg.angular.x = 0;
	vel_msg.angular.y = 0;

	
	//Kw value must be adjusted carefully a little by little, trying theta* = theta
	//Large Kw value may cause strange behavior due to too big change of the turtle’s orientation.
	double Kw = 1.1;

	//**********need to figure out the angle for this to go around Xturtles
	vel_msg.angular.z = Kw*(atan2(t2 - turtlesim_pose.y, t1 - turtlesim_pose.x)
	- turtlesim_pose.theta); // Kw(θ * - θ)
	//angular.z is the relative angle to rotate calculated by (absolute_angle – turtle’s orientation)

	//****to prevent offbounds (0,0) (11,11)
	if (vel_msg.linear.y > 0 || vel_msg.linear.x > 0 || vel_msg.linear.y < 11 || vel_msg.linear.x < 11)
		velocity_publisher.publish(vel_msg);
	//***CURRENTLY TRYING TO THINK OF CONDITIONS TO MAKE OUR TURTLE WALK AROUND XTURTLES

	//******go to turtle2 when our turtle is at turtle1
	if (getDistance(turtlesim_pose.x, turtlesim_pose.y, x, y) >= 0.5){
		t1 = x1;
		t2 = y1;
	}
	ros::spinOnce();
	loop_rate.sleep();
	
	//******finish when our turtle is at turtle2
	} while(getDistance(turtlesim_pose.x, turtlesim_pose.y, x1, y1) >= 0.5); // NEED THIS || or turtle is dead
	std::cout<<"end move goal"<<std::endl;

	vel_msg.linear.x = 0;
	vel_msg.angular.z = 0;
	velocity_publisher.publish(vel_msg);
}
int main(int argc, char **argv){

	ros::init(argc,argv,"hw4");
	ros::NodeHandle n;
	int turtle1 = 1; // Store Tturtle1
	int turtle2 = 1; // Store Tturtle2
	double max_distance = 0; // To check the furthest pair
	double distance = 0; // To check pair distances
	stringstream jname; 
	stringstream tname;

	velocity_publisher = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel",1000);
	pose_subscriber = n.subscribe("/turtle1/pose",10,poseCallback);

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
	ROS_INFO("The furthest pairs are %d at %f , %f and %d at %f , %f",(turtle1+1),t_turts[turtle1].x, t_turts[turtle1].y 
	,(turtle2+1), t_turts[turtle2].x, t_turts[turtle2].y);

	moveGoal(t_turts[turtle1].x, t_turts[turtle1].y, t_turts[turtle2].x, t_turts[turtle2].y);

   return 0;
}
