//robot_cleaner.cpp
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "turtlesim/Pose.h"
#include <sstream>
using namespace std;

ros::Publisher velocity_publisher;
ros::Subscriber pose_subscriber;
turtlesim::Pose turtlesim_pose;
const double x_min = 0.0;
const double y_min = 0.0;
const double x_max = 11.0;
const double y_max = 11.0;
const double PI = 3.14159265359;

//to move from one point to specified distance
void move(double speedx, double speedy, double distance, bool isForward){
	geometry_msgs::Twist vel_msg;

	//set a random linear velocity in the x-axis
	if (isForward){
		vel_msg.linear.x = abs(speedx);
		vel_msg.linear.y = abs(speedy);
		vel_msg.linear.z = abs(speedy);
}
	else{
		vel_msg.linear.x = -abs(speedx);
		vel_msg.linear.y = -abs(speedy);
		vel_msg.linear.z = -abs(speedy);
}
	

	//set a random angular velocity in teh y-axis
	vel_msg.angular.x = 0;
	vel_msg.angular.y = 0;
	vel_msg.angular.z = 0;

	//t0: current time
	double t0 = ros::Time::now().toSec();
	double current_distance = 0.0;
	ros::Rate loop_rate(100);
	double t1;
	do{
	//publish the msg
		velocity_publisher.publish(vel_msg);
		t1 = ros::Time::now().toSec();
		
		if (speedy != 0){
			current_distance = speedy *(t1-t0);
ROS_INFO_STREAM((t1-t0) << "," << current_distance << "," << distance << "," << vel_msg.linear.z << endl);
}
		else {
			current_distance = speedx *(t1-t0);
ROS_INFO_STREAM((t1-t0) << "," << current_distance << "," << distance << "," << vel_msg.linear.x << endl);
}
		ros::spinOnce();
		loop_rate.sleep();
		
	}while(current_distance < distance);
	vel_msg.linear.x = 0;
	vel_msg.linear.y = 0;
	vel_msg.linear.z = 0;
	velocity_publisher.publish(vel_msg);


}
void poseCallback(const turtlesim::Pose::ConstPtr &pose_message){
	turtlesim_pose.x = pose_message -> x;
	turtlesim_pose.y = pose_message -> y;
	turtlesim_pose.theta = pose_message -> theta;
}

void rotate(double angular_speed,double relative_angle, bool clockwise){
geometry_msgs::Twist vel_msg;
	//set a random linear velocity in the x-axis
	vel_msg.linear.x = 0;
	vel_msg.linear.y = 0;
	vel_msg.linear.z = 0;
	//set a random angular velocity in the y-axis
	vel_msg.angular.x = 0;
	vel_msg.angular.y = 0;

	if (clockwise)
		vel_msg.angular.z = -abs(angular_speed);
	else
		vel_msg.angular.z = abs(angular_speed);
	double current_angle = 0.0;
	double t0 = ros::Time::now().toSec();
	double t1;
	ros::Rate loop_rate(1000);
	do{
		velocity_publisher.publish(vel_msg);
		t1 = ros::Time::now().toSec();
		current_angle = angular_speed * (t1 - t0);
		ros::spinOnce();
		loop_rate.sleep();
	}while(current_angle < relative_angle);

	vel_msg.angular.z = 0;
	velocity_publisher.publish(vel_msg);
}
double degrees2radians(double angle_in_degrees){
	return angle_in_degrees * PI / 180.0;
}

double setDesiredOrientation (double desired_angle_radians){
	double relative_angle_radians = desired_angle_radians - turtlesim_pose.theta;
	bool clockwise = ((relative_angle_radians<0)?true:false);
	cout << desired_angle_radians << "," << turtlesim_pose.theta << ","<<relative_angle_radians;
	rotate (abs(relative_angle_radians),abs(relative_angle_radians),clockwise);
}

double getDistance(double x1, double y1, double x2, double y2){
	return sqrt(pow((x1-x2),2) + pow((y1-y2),2));
}

void moveGoal(turtlesim::Pose goal_pose, double distance_tolerance){
	geometry_msgs::Twist vel_msg;
	ros::Rate loop_rate(100);
	double E = 0.0;
	do{
		double Kv = 1.0;
		//double kl = 0.02;
		//double v0 = 2.0;
		//double alpha = 0.5;
		
		//getDistance calculates Euclidean distance
		double e = getDistance(turtlesim_pose.x,turtlesim_pose.y,goal_pose.x,goal_pose.y);
		//double E = E + e;		
		
		//Kv = v0 * (exp(-alpha)*error*error)/(error*error);//try something else
		vel_msg.linear.x = (Kv*e);
		vel_msg.linear.y = 0;
		vel_msg.linear.z = 0;

		//angular velocity in the z-axis
		vel_msg.angular.x = 0;
		vel_msg.angular.y = 0;
		double Kw = 1.5;
	
		//Kw value must be adjusted carefully trying theta
		//Large Kw value may cause strange behavior 

		vel_msg.angular.z = Kw*(atan2(goal_pose.y - turtlesim_pose.y,goal_pose.x - turtlesim_pose.x) - turtlesim_pose.theta);
		//angular.z is relative angle to rotate calculated by (abs_angle - turtle orientation)
		velocity_publisher.publish(vel_msg);
		ros::spinOnce();
		loop_rate.sleep();
	}while(getDistance(turtlesim_pose.x,turtlesim_pose.y,goal_pose.x,goal_pose.y) > distance_tolerance);

	cout << "end move goal" << endl;

	vel_msg.linear.x = 0;
	vel_msg.angular.z = 0;
	velocity_publisher.publish(vel_msg);
}


int main(int argc, char **argv){
	ros::init(argc,argv,"turtlesim_cleaner");
	ros::NodeHandle n;
	double speed,angular_speed;
	double distance, angle;
	bool isForward, clockwise;
	velocity_publisher = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel",1000);
	pose_subscriber = n.subscribe("/turtle1/pose",10,poseCallback);

	ros::Rate loop_rate(20);
	ROS_INFO_STREAM("\n\n\n*****START MOVING *******\n");
	speed = 0.2;
	distance = 2;
	isForward = true;


//MAKE THE NUMBER 7
	angular_speed = 2.0;
	angle = 90.0;
	clockwise = false;
	rotate(degrees2radians(angular_speed), degrees2radians(angle),clockwise);
	move(0.5,0.4,1, isForward);
	clockwise = true;
	rotate(degrees2radians(angular_speed), degrees2radians(angle),clockwise);
	move(0.2,0,distance, isForward);
	clockwise = false;
	rotate(degrees2radians(angular_speed), degrees2radians(angle),clockwise);
	isForward = false;
	move(0.5,0.4,2, isForward);
	//ros::Rate loop_rate(0.5);
	//loop_rate.sleep();
	//setDesiredOrientation(degrees2radians(60));
	//loop_rate.sleep();
	//setDesiredOrientation(degrees2radians(60));

	/*turtlesim::Pose pose;
	pose.x = 1;
	pose.y = 1;
	pose.theta = 0;
	moveGoal(pose,0);*/
	loop_rate.sleep();
	ros::spin();
	return 0;
}
