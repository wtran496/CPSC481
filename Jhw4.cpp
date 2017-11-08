#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "turtlesim/Pose.h"
#include <turtlesim/Spawn.h>
#include <turtlesim/Kill.h>
#include <sstream>
#include <string>
#include <boost/algorithm/string.hpp>

using namespace::std;

struct Turtle {
	string name;
	turtlesim::Pose pose;
};
const int MAX_TTURTLES = 7;
const int MAX_XTURTLES = 10;
turtlesim::Pose turtle1_s;
int tCount = 0;
int xCount = 0;
static ros::ServiceClient kClient;

double getDistance(double x1, double y1, double x2, double y2){
	return sqrt(pow((x1-x2),2) + pow((y1-y2),2));
}

string getTurtlename(const string topicname) {
  vector<string> elems;
  char lc_delim[2];
  lc_delim[0] = '/';
  lc_delim[1] = '\0';

  boost::algorithm::split(elems, topicname, boost::algorithm::is_any_of(lc_delim));
  return elems[1];
}

bool turtleExist(const string turtlename) {
  int i;
  string tname;
  ros::master::V_TopicInfo alltopics;

  //get all topic names
  ros::master::getTopics(alltopics);

  for (int i=0; i<alltopics.size(); i++) {
     tname = getTurtlename(alltopics[i].name);
     if (tname.compare(turtlename) == 0) {
        return true;
     };
  };
  return false;
}

void avoidCollision(ros::NodeHandle n)
{
    geometry_msgs::Twist vel_msg;
    ros::Rate loop_rate(100);
    ROS_WARN("Avoiding collision");
    // stop
    vel_msg.linear.x = 0;
    vel_msg.linear.y = 0;
    vel_msg.linear.z = 0;

    vel_msg.angular.x = 0;
    vel_msg.angular.y = 0; //to be safe
    vel_msg.angular.z = 0; //to be safe

	ros::Publisher velocity_publisher;
	velocity_publisher = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel",1000);
    velocity_publisher.publish(vel_msg);

    // Drive backwards and turn to the left or right

    vel_msg.linear.x = -1;
    velocity_publisher.publish(vel_msg);
    vel_msg.linear.x = 0.0;
    sleep(2);

    int direction = rand() % 2;
    double rotate = 0;
    vel_msg.linear.x = 0.0;
    double current_angle = 0;
    //choose a position to rotate
    if(direction)
    {
        vel_msg.angular.z = -3.14/2;
		rotate = vel_msg.angular.z;
    }
    else
    {
    	vel_msg.angular.z = -3.14/2;
		rotate = vel_msg.angular.z;
    }
	double t0 = ros::Time::now().toSec();
	double t1;
	//rotate

	do{
		velocity_publisher.publish(vel_msg);
		t1 = ros::Time::now().toSec();
		current_angle = rotate * (t1 - t0);
		ros::spinOnce();
		loop_rate.sleep();
		ROS_INFO("Rotation at %f", current_angle);
	}while(current_angle < 1 && current_angle > -1);

    vel_msg.angular.z = 0;
    velocity_publisher.publish(vel_msg);
}

void move_to(turtlesim::Pose dest, turtlesim::Pose x_turts[], string t_name, ros::NodeHandle n){

	// Tuner variables
	float k_linear = 1;
	float k_angular = 6;
	double dist;
	// To handle kill client/service requests
	turtlesim::Kill::Request reqk;
	turtlesim::Kill::Response respk;
	kClient = n.serviceClient<turtlesim::Kill>("kill");
	reqk.name = t_name;

	// ROS variable set up
	ros::Publisher velocity_publisher = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel",1000);
	geometry_msgs::Twist vel_msg;
	ros::Rate loop_rate(50);
	vel_msg.linear.y = 0;
	vel_msg.linear.z = 0;
	vel_msg.angular.x = 0;
	vel_msg.angular.y = 0;

	// Navigate to destination
	while(getDistance(turtle1_s.x, turtle1_s.y, dest.x, dest.y) > .5){
		dist = getDistance(turtle1_s.x, turtle1_s.y, dest.x, dest.y);
		vel_msg.linear.x = dist * k_linear;
		cout<<"Goal: "<<(atan2(dest.y - turtle1_s.y, dest.x - turtle1_s.x))<<endl;
		cout<<"Current: "<< turtle1_s.theta<<endl;
		vel_msg.angular.z = k_angular * (atan2(dest.y - turtle1_s.y, dest.x - turtle1_s.x)
		- turtle1_s.theta);

		velocity_publisher.publish(vel_msg);

		// Checking for close X turts
		for( int i = 0; i < xCount; i++){
			double distX = getDistance(turtle1_s.x, turtle1_s.y, x_turts[i].x, x_turts[i].y);
			//if our turtle is within the boundary of one of the X turtle then avoid
			if (distX <= 1 && vel_msg.linear.y >= 0 && vel_msg.linear.x >= 0 && vel_msg.linear.y <= 11 && vel_msg.linear.x <= 11) {
				avoidCollision(n);
			}
		}

		ros::spinOnce();
		loop_rate.sleep();
	}
	if (!kClient.call(reqk, respk))
		ROS_ERROR_STREAM("Error: Failed to kill " << reqk.name.c_str() << "\n");
	else
		ROS_INFO_STREAM("Turtle captured!");
}

void navigate(Turtle t_turts[], turtlesim::Pose x_turts[], ros::NodeHandle n){

	ros::Publisher velocity_publisher = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel",1000);
	ros::Rate loop_rate(1);

	double distance_t1, distance_t2;
	geometry_msgs::Twist vel_msg;
	Turtle t1, t2;
	t1 = t_turts[0];
	t2 = t_turts[1];

	distance_t1 = getDistance(turtle1_s.x, turtle1_s.y, t1.pose.x, t1.pose.y);
	distance_t2 = getDistance(turtle1_s.x, turtle1_s.y, t2.pose.x, t2.pose.y);
	cout<< "D1: "<< distance_t1 <<"\tD2: "<< distance_t2 << endl;

	if (distance_t1 > distance_t2){
		move_to(t1.pose, x_turts, t1.name, n);
		loop_rate.sleep();
		loop_rate.sleep();
		move_to(t2.pose, x_turts, t2.name, n);
	}
	else{
		move_to(t2.pose, x_turts, t2.name, n);
		loop_rate.sleep();
		loop_rate.sleep();
		move_to(t1.pose, x_turts, t1.name, n);
	}
}



void poseCallback(const turtlesim::Pose::ConstPtr &pose_message){
	turtle1_s.x = pose_message -> x;
	turtle1_s.y = pose_message -> y;
	turtle1_s.theta = pose_message -> theta;
}

int main(int argc, char **argv){
	int turtle1 = 1; // Store Tturtle1
	int turtle2 = 1; // Store Tturtle2
	double max_distance = 0; // To check the furthest pair
	double distance = 0; // To check pair distances

	ros::init(argc,argv,"hw4");
	ros::NodeHandle n;
	ros::Subscriber pose_subscriber = n.subscribe("/turtle1/pose",10,poseCallback);
	ros::Rate loop_rate(2);

	turtlesim::Pose turtle_1;
	/*
		COLLECTING A LIST OF AVAILABLE TURTLES
	*/
	for( int i = 1; i < MAX_TTURTLES + 1; i++){
		stringstream name;
		name << "T" << i;
		cout<<"Checking "<<name.str()<<endl;
		bool result = turtleExist(name.str());
		if (result){
			cout<<name.str()<<" exists!"<<endl;
			tCount++;
		}
	}

	for( int i = 1; i < MAX_TTURTLES + 1; i++){
		stringstream name;
		name << "X" << i;
		cout<<"Checking "<<name.str()<<endl;
		bool result = turtleExist(name.str());
		if (result){
			cout<<name.str()<<" exists!"<<endl;
			xCount++;
		}
	}

	/*
		COLLECTING ALL TURTLE LOCATIONS
	*/
	// Two arrays to store locations of Xs and Ts
	turtlesim::Pose x_turts[xCount];
	turtlesim::Pose t_turts[tCount];
	turtlesim::Pose turtle_position;
	ros::Duration wait_duration = ros::Duration(2);
	stringstream tname;
	string name;

	for( int i = 0; i < tCount; i++){
		tname.clear();
		tname.str("");
		tname << "/T" << i+1 << "/pose";
		name = tname.str();
		// We can use waitForMessage because we previously identified which turtles exist
		t_turts[i] = *(ros::topic::waitForMessage<turtlesim::Pose>(name, wait_duration));
	}
	for( int i = 0; i < xCount; i++){
		tname.clear();
		tname.str("");
		tname << "/X" << i+1 << "/pose";
		name = tname.str();
		// We can use waitForMessage because we previously identified which turtles exist
		x_turts[i] = *(ros::topic::waitForMessage<turtlesim::Pose>(name, wait_duration));
	}

	/*
			GETTING FURTHEST PAIR OF T-TURTLES
	*/
	for(int src_turt = 0; src_turt  < tCount; src_turt ++ ){
		cout<<"\nOuter loop:  Turtle--"<<src_turt+1<<endl<<endl;	// Added this for personal visualization purposes-- Delete if you want
		float src_x = t_turts[src_turt].x;
		float src_y = t_turts[src_turt].y;

		// src_turt+1 to compare each turtle
		for (int dest_turt = src_turt+1; dest_turt < tCount; dest_turt++){
			cout<<"\tInner loop:  Turtle--"<<dest_turt+1<<endl;	// Added this for personal 		move_to(t1,n);
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

	Turtle goal_turts[2];

	tname.clear();
	tname.str("");
	tname << "T" << turtle1 + 1;
	name = tname.str();

	goal_turts[0].pose = t_turts[turtle1];
	goal_turts[0].name = name;

	tname.clear();
	tname.str("");
	tname << "T" << turtle2 + 1;
	name = tname.str();

	goal_turts[1].pose = t_turts[turtle2];
	goal_turts[1].name = name;
	navigate( goal_turts, x_turts , n);
   return 0;
}

