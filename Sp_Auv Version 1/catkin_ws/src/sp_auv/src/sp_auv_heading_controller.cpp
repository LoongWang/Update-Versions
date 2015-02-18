#define DEPTH_THURST_OFFSET 32
#define THRUST_LIMIT 100

#include <iostream>
#include <stdio.h>
#include <string>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Pose2D.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Int16.h"

float heading;
float depth;
int count;
std::string KS;

float des_depth;
float des_heading;
float des_thrust;

float depth_tolerance;
float depth_kp;
float heading_kp;
std::string test;
int clamp(int v, int minv, int maxv)
{
    return std::min(maxv,std::max(minv, v));
}


void CompassCallback(const geometry_msgs::Pose2D::ConstPtr& msg)
{
  //ROS_INFO("x: [%f]", msg->x);
  //ROS_INFO("y: [%f]", msg->y);
  //ROS_INFO("theta: [%f]", msg->theta);
  heading = msg->theta;
}

void SounderCallback(const std_msgs::Float32::ConstPtr& msg)
{
  //ROS_INFO("depth: [%2.1f]", msg->data);
  depth = msg->data;
}

void DesDepthCallback(const std_msgs::Float32::ConstPtr& msg)
{
  des_depth = msg->data;
}

void DesHeadingCallback(const std_msgs::Float32::ConstPtr& msg)
{
  des_heading = msg->data;
}

void DesThrustCallback(const std_msgs::Float32::ConstPtr& msg)
{
  des_thrust = msg->data;
}

void KillswitchCallback(const std_msgs::String::ConstPtr& msg)
{
  KS =  msg->data.c_str();
}


int main(int argc, char **argv)
{
	

  ros::init(argc, argv, "controller");

  ros::NodeHandle n;
  std_msgs::Int16 thrust_hl, thrust_hr, thrust_vl, thrust_vr;

  ros::Subscriber sub_compass = n.subscribe("SpartonCompass", 1000, CompassCallback);
  ros::Subscriber sub_sounder = n.subscribe("DepthSounder", 1000, SounderCallback);
  ros::Subscriber sub_des_dep = n.subscribe("DesiredDepth", 1000, DesDepthCallback);
  ros::Subscriber sub_des_hed = n.subscribe("DesiredHeading", 1000, DesHeadingCallback);
  ros::Subscriber sub_des_thr = n.subscribe("DesiredThrust", 1000, DesThrustCallback);
  ros::Subscriber sub_killswitch = n.subscribe("Estop", 1000, KillswitchCallback);
  ros::Publisher pub_hl = n.advertise<std_msgs::Int16>("thrust_hl", 500);
  ros::Publisher pub_hr = n.advertise<std_msgs::Int16>("thrust_hr", 500);
  ros::Publisher pub_vl = n.advertise<std_msgs::Int16>("thrust_vl", 500);
  ros::Publisher pub_vr = n.advertise<std_msgs::Int16>("thrust_vr", 500);
  
  ros::NodeHandle nh("~");
  double getval;

  nh.param("DepthTolerance",getval,.1);
  depth_tolerance = (float) getval;
  ROS_ERROR("Out: %2.1f",depth_tolerance);

  nh.param("DepthKP",getval,-10.0);
  depth_kp = (float) getval;

  nh.param("HeadingKP",getval,3.0);
  heading_kp = (float) getval;



  ros::Rate loop_rate(10);

  while (ros::ok()) {

     ros::spinOnce();
    
     //Logging file
    FILE * pfile;
    pfile = fopen("Depth_results.txt","a");
    fprintf(pfile," \n");
    fprintf(pfile,"Start of Depth Kp test:\n");
    fprintf(pfile,"Sonar value: %2.1f\n", depth);
	
	if ((des_depth != 0.0) && (KS == "GO!"))
 	{

	ROS_INFO("Des depth: [%2.1f]", des_depth);
        // Depth Control
	float depth_E = depth - des_depth; //DESIRED_DEPTH;
      
	ROS_INFO("Depth Error: [%2.1f]", depth_E);

     		if ((depth_E*depth_E) > ( depth_tolerance * depth_tolerance )) 
      		{
        		 //thrust_vr.data=thrust_vl.data = clamp((depth_kp * depth_E) + DEPTH_THURST_OFFSET,-THRUST_LIMIT,THRUST_LIMIT);
			thrust_vr.data=thrust_vl.data=0;	
      		}
      		else 
      		{
          		//thrust_vr.data=thrust_vl.data = clamp(DEPTH_THURST_OFFSET,-THRUST_LIMIT,THRUST_LIMIT);
       			thrust_vr.data=thrust_vl.data=0;
		}	
 	}
	else	{	thrust_vr.data=thrust_vl.data=0;	}
   
          pub_vr.publish(thrust_vr);
          pub_vl.publish(thrust_vl);
          ROS_INFO("Vert Thrust: [%d]", thrust_vr.data);

     // Heading Control
      float heading_E = heading - des_heading ;//DESIRED_HEADING;
      float heading_D = (heading_kp * heading_E); 
     
	//if((thrust_vr.data == DEPTH_THURST_OFFSET ) && (thrust_vl.data == DEPTH_THURST_OFFSET ))
	//{	
		//Compass: (clockwise - > negative value) & (anticlockwise -> positve value)
		//Moving from anticlockwise position , decrease right motor //Moving from clockwise position, decrease your left
		thrust_hr.data =  clamp( des_thrust + heading_D,-THRUST_LIMIT,THRUST_LIMIT);		
		thrust_hl.data =  clamp( des_thrust - heading_D,-THRUST_LIMIT,THRUST_LIMIT);
	//}
	//else	{	thrust_hr.data=thrust_hl.data=0;	}

      ROS_INFO("Heading Error: [%2.1f]", heading_E);
      ROS_INFO("HorL Thrust: [%d]", thrust_hl.data);
      ROS_INFO("HorR Thrust: [%d]", thrust_hr.data);

      pub_hr.publish(thrust_hr);
      pub_hl.publish(thrust_hl);
       
       fprintf(pfile,"Vert thrust value: %d\n", thrust_vr.data);
       fprintf(pfile,"Horzl thrust value: %d\n", thrust_hl.data);
      loop_rate.sleep();

  }



  return 0;
}

