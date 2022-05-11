#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <iostream>
#include "std_msgs/Int16MultiArray.h"
#include <math.h>

using namespace std;
std_msgs::Int16MultiArray Info_rcv,trigger_rcv;
geometry_msgs::Twist vel;
int flag,trigger;
int clear_flag = 1;
int clear_counter = 0;
int Open_counter = 450;
const int safe_time = 100;

void trigger_callback(const std_msgs::Int16MultiArray msg)
	{
             trigger_rcv = msg;
             trigger = int(trigger_rcv.data.at(0));
	     return;
	}	

void flag_callback(const std_msgs::Int16MultiArray msg)
        {
	     Info_rcv = msg;
              
	     flag = int(Info_rcv.data.at(0));


             //cout << flag << endl;
	     
             return;
        }
int main(int argc, char **argv){
	ros::init(argc, argv, "mini_node");
	ros::NodeHandle nh;
	ros::Publisher pub_vel = nh.advertise<geometry_msgs::Twist>("cmd_vel", 10); 
	ros::Subscriber Flag_subs = nh.subscribe<std_msgs::Int16MultiArray>("obstacle_flag", 1000, flag_callback);
	ros::Subscriber Trigger_subs = nh.subscribe<std_msgs::Int16MultiArray>("trigger_flag",1000,trigger_callback);
        ros::Rate rate(20.0);	
        
       	
	while (ros::ok()){
	       	
	       if (flag == 1){
		    clear_flag = 0;
		    clear_counter = 1;
	      	}       
	       
	       if (clear_counter >=1 && clear_counter < safe_time){
		         clear_counter = clear_counter + 1;
		         clear_flag = 0;
	               }
	       if (clear_counter == safe_time){
		         clear_flag = 1;
		         clear_counter = 0;  
	               }
	       
               cout << "trigger = " << trigger << endl;
	       cout << "clear_counter=" << clear_counter <<endl;
	       cout << "clear_flag = " << clear_flag << endl;	
               cout << "Open_counter=" << Open_counter <<endl;
               if (trigger == 1 && clear_flag == 1 && Open_counter > 0 ){ 
                     
		     vel.linear.x = 0.15; 
                     vel.linear.y = 0;
	 	     vel.linear.z = 0;
		     vel.angular.x = 0;
	             vel.angular.y = 0;	
                     vel.angular.z = 0.05;
                		
	             
		     Open_counter = Open_counter - 1;
	          }   
	       else {
                     vel.linear.x = 0;
		     vel.linear.y = 0;
		     vel.linear.z = 0;
		     vel.angular.x = 0;
		     vel.angular.y = 0;
		     vel.angular.z = 0;
                     
                     		     
                    }
	       	       
	        pub_vel.publish(vel);
		ros::spinOnce(); 
		rate.sleep();
	}

        return 0;
}	
