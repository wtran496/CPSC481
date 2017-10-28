#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "turtlesim/Pose.h"
#include <sstream>
#include <turtlesim/Spawn.h>
#include <turtlesim/Kill.h>
#include <string>
#include <sstream>

using namespace::std;

double getDistance(double x1, double y1, double x2, double y2){
return sqrt(pow((x1-x2),2) + pow((y1-y2),2));
}
int main(int argc, char **argv){

   	ros::init(argc,argv,"hw4");
  	ros::NodeHandle n;
	turtlesim::Pose msg;	//  Defines the message obj
	stringstream tname;
	ros::Rate loop_rate(2);
	int turts = 5;
	ros::Subscriber Tturts[turts];
	double pair = 0; //to check the furthest pair
	turtlesim::Pose msg1;
	stringstream jname; 
	double distance = 0; //check pair distances
	int turtle1 = 1; //store Tturtle1
	int turtle2 = 1; //store Tturtle2

	for(int i = 0; i < turts; i ++ ){
		tname.clear();
		tname.str("");
		tname << "/T" << i+1 << "/pose";
		string name = tname.str();
		try{msg = *(ros::topic::waitForMessage<turtlesim::Pose>(name, ros::Duration(2)));}
		catch (int e){ROS_INFO("COULDNT FIND THAT TURT");}
		ROS_INFO("X: %f,  Y: %f", msg.x, msg.y);
		//i+1 to compare each turtle
		for (int j = i+1; j < turts; j++){
			jname.clear();
			jname.str("");
			jname << "/T" << j+1 << "/pose";
			string name = jname.str();

			try{msg1 = *(ros::topic::waitForMessage<turtlesim::Pose>(name, ros::Duration(2)));}
			catch (int e){ROS_INFO("COULDNT FIND THAT TURT");}
			ROS_INFO("X1: %f,  Y1: %f", msg1.x, msg1.y);
			//get the distance
			double distance = getDistance(msg.x,msg.y,msg1.x,msg1.y);
			//check the distance
			ROS_INFO("Distance: %f",distance);
			if (distance > pair){
				//Check the pair
				ROS_INFO("PAIR AND DISTANCE: %f , %f",pair,distance);
				turtle1 = i+1;
				turtle2 = j+1;
				pair = distance;
			}
		}
	}

	//This whole section display the pairs
	tname.clear();
	tname.str("");
	tname << "/T" << turtle1 << "/pose";
	string name = tname.str();
	try{msg = *(ros::topic::waitForMessage<turtlesim::Pose>(name, ros::Duration(2)));}
	catch (int e){ROS_INFO("COULDNT FIND THAT TURT");}
	name.clear();
		
	jname.str("");
	jname << "/T" << turtle2 << "/pose";		
	name = jname.str();
	try{msg1 = *(ros::topic::waitForMessage<turtlesim::Pose>(name, ros::Duration(2)));}
	catch (int e){ROS_INFO("COULDNT FIND THAT TURT");}
	name.clear();

	cout << "The furthest pairs are " << turtle1 << " at " << msg.x << "," << msg.y << " and " << turtle2 << " at " << msg1.x << "," << msg1.y;
   return 0;
}
