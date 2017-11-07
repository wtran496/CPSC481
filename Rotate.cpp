#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "turtlesim/Pose.h"
#include <turtlesim/Spawn.h>
#include <turtlesim/Kill.h>
#include <sstream>
#include <string>
#include <boost/algorithm/string.hpp>

const int MAX_TTURTLES = 7;
const int MAX_XTURTLES = 10;
turtlesim::Pose turtle1_s;
ros::Publisher velocity_publisher;

using namespace::std;

void poseCallback(const turtlesim::Pose::ConstPtr &pose_message){
	turtle1_s.x = pose_message -> x;
	turtle1_s.y = pose_message -> y;
	turtle1_s.theta = pose_message->theta; 
}
void avoidCollision()
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

    velocity_publisher.publish(vel_msg);

    // Drive backwards and turn to the left or right

    vel_msg.linear.x = -0.5;
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
        vel_msg.angular.z = -10;    
	rotate = vel_msg.angular.z;
    }
    else
    {
    	vel_msg.angular.z = 10;
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

void navigate(turtlesim::Pose t_turts[], ros::NodeHandle n,turtlesim::Pose x_turts[],int xCount,double pairs[]){

	velocity_publisher = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel",1000);
	double distance_t1, distance_t2;
	double time1 = ros::Time::now().toSec();
	double time2 = ros::Time::now().toSec();
	double time = 0;
	geometry_msgs::Twist vel_msg;
	turtlesim::Pose t1, t2;
	t1 = t_turts[0];
	t2 = t_turts[1];
	distance_t1 = getDistance(turtle1_s.x, turtle1_s.y, t1.x, t1.y);
	distance_t2 = getDistance(turtle1_s.x, turtle1_s.y, t2.x, t2.y);

	//display euclidean distance between the t turtles
	double euclidDistance = getDistance(t1.x,t1.y,t2.x,t2.y);
	cout<< "D1: "<< distance_t1 <<"\tD2: "<< distance_t2 << endl;
	ros::Rate loop_rate(100);

	// Was just testing sending things --probably change this to no longer be a for loop
	vel_msg.linear.y = 0;
	vel_msg.linear.z = 0;
	vel_msg.angular.x = 0;
	vel_msg.angular.y = 0;

	//getDistance calculates Euclidean distance
	// Tuner variables
	float k_linear = 1;
	float k_angular = 4;
	double dist = getDistance(turtle1_s.x, turtle1_s.y, t1.x, t1.y);
	while(dist >= 0.5){
		dist = getDistance(turtle1_s.x, turtle1_s.y, t1.x, t1.y);

		//getting the time
		time2 = ros::Time::now().toSec();
		time = time2 - time1;
		ROS_INFO("Time: %f",time);

		ROS_INFO("Distance is now %f",dist);
		vel_msg.linear.x = dist * k_linear;
		vel_msg.angular.z = k_angular * (atan2(t1.y - turtle1_s.y, t1.x - turtle1_s.x)
		- turtle1_s.theta);

		velocity_publisher.publish(vel_msg);
	//check all x turtle locations
	for( int i = 0; i < xCount; i++){
	double distX = getDistance(turtle1_s.x, turtle1_s.y, x_turts[i].x, x_turts[i].y);
	//if our turtle is within the boundary of one of the X turtle then avoid
	if (distX <= 1 && vel_msg.linear.y >= 0 && vel_msg.linear.x >= 0 && vel_msg.linear.y <= 11 && vel_msg.linear.x <= 11) {
            avoidCollision();
        	}
}
		ros::spinOnce();
		loop_rate.sleep();
	}
	dist = getDistance(turtle1_s.x, turtle1_s.y, t2.x, t2.y);
	while(dist >= 0.5){
		dist = getDistance(turtle1_s.x, turtle1_s.y, t2.x, t2.y);
	
		//getting the time
		time2 = ros::Time::now().toSec();
		time = time2 - time1;
		ROS_INFO("Time: %f",time);

		ROS_INFO("Distance2 is now %f",dist);
		vel_msg.linear.x = dist * k_linear;
		vel_msg.angular.z = k_angular * (atan2(t2.y - turtle1_s.y, t2.x - turtle1_s.x)
		- turtle1_s.theta);

		velocity_publisher.publish(vel_msg);
	for( int i = 0; i < xCount; i++){
	double distX = getDistance(turtle1_s.x, turtle1_s.y, x_turts[i].x, x_turts[i].y);
	if (distX <= 1 && vel_msg.linear.y >= 0 && vel_msg.linear.x >= 0 && vel_msg.linear.y <= 11 && vel_msg.linear.x <= 11) {
            avoidCollision();
        	}
}
		ros::spinOnce();
		loop_rate.sleep();
	}
	ROS_INFO("Time: %f",time);
//find the total distance
	double totalDistance = vel_msg.linear.x * time;

//display list
	ROS_INFO("T Turtles pair: %f and %f\n Euclidean Distance between T turtles: %f\n Velocity: %f\n Total Time: %f\n Total Distance: %f\n",pairs[0],pairs[1],euclidDistance,vel_msg.linear.x,time,totalDistance);
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

	double pairing[2];

	/*
			GETTING FURTHEST PAIR OF T-TURTLES
	*/
	for(int src_turt = 0; src_turt  < tCount; src_turt ++ ){
		cout<<"\nOuter loop:  Turtle--"<<src_turt+1<<endl<<endl;	// Added this for personal visualization purposes-- Delete if you want
		float src_x = t_turts[src_turt].x;
		float src_y = t_turts[src_turt].y;

		// src_turt+1 to compare each turtle
		for (int dest_turt = src_turt+1; dest_turt < tCount; dest_turt++){
			cout<<"\tInner loop:  Turtle--"<<dest_turt+1<<endl;	// Added this for personal visualization purposes-- Delete if you want
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
				pairing[0] = src_turt;
				pairing[1] = dest_turt;
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
//						pairing is to get the turtle number pair
	navigate(t_turts, n, x_turts, xCount,pairing);
   return 0;
}
