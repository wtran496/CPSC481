#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "turtlesim/Pose.h"
#include <turtlesim/Spawn.h>
#include <turtlesim/Kill.h>
#include <sstream>
#include <string>
#include <sstream>
#include <boost/algorithm/string.hpp>

const int MAX_TTURTLES = 7;
const int MAX_XTURTLES = 10;
turtlesim::Pose turtle1_s;

using namespace::std;

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


void navigate(turtlesim::Pose t_turts[], ros::NodeHandle n){

	ros::Publisher velocity_publisher = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel",1000);

	double distance_t1, distance_t2;
	geometry_msgs::Twist vel_msg;
	turtlesim::Pose t1, t2;
	t1 = t_turts[0];
	t2 = t_turts[1];

	distance_t1 = getDistance(turtle1_s.x, turtle1_s.y, t1.x, t1.y);
	distance_t2 = getDistance(turtle1_s.x, turtle1_s.y, t2.x, t2.y);
	cout<< "D1: "<< distance_t1 <<"\tD2: "<< distance_t2 << endl;
	ros::Rate loop_rate(100);

	// Was just testing sending things --probably change this to no longer be a for loop
	for(int i = 0; i < 100; i++){

		vel_msg.linear.x = 1.5 * distance_t1;
		vel_msg.linear.y = 0;
		vel_msg.linear.z = 0;

		vel_msg.angular.x = 0;
		vel_msg.angular.y = 0;
		vel_msg.angular.z = 1.1*(atan2(t1.y - turtle1_s.y, t1.x - turtle1_s.x)
			- turtle1_s.theta); // Kw(θ * - θ)

		velocity_publisher.publish(vel_msg);
		ros::spinOnce();
		loop_rate.sleep();
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
	int tCount = 0;
	int xCount = 0;
	double max_distance = 0; // To check the furthest pair
	double distance = 0; // To check pair distances

	ros::init(argc,argv,"hw4");
	ros::NodeHandle n;
	ros::Subscriber pose_subscriber = n.subscribe("/turtle1/pose",10,poseCallback);
	ros::Rate loop_rate(2);

	turtlesim::Pose turtle_1;
	/*
		COLLECTING ALL TURTLE LOCATIONS
	*/
	for( int i = 0; i < MAX_TTURTLES; i++){
		stringstream name;
		name << "T" << i;
		cout<<"Checking "<<name.str()<<endl;
		bool result = turtleExist(name.str());
		if (result){
			cout<<name.str()<<" exists!"<<endl;
			tCount++;
		}
	}

	for( int i = 0; i < MAX_TTURTLES; i++){
		stringstream name;
		name << "X" << i;
		cout<<"Checking "<<name.str()<<endl;
		bool result = turtleExist(name.str());
		if (result){
			cout<<name.str()<<" exists!"<<endl;
			xCount++;
		}
	}

	turtlesim::Pose x_turts[xCount];
	turtlesim::Pose t_turts[tCount];


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

	turtlesim::Pose goal_turts[2];
	goal_turts[0] = t_turts[turtle1];
	goal_turts[1] = t_turts[turtle2];
   return 0;
}

