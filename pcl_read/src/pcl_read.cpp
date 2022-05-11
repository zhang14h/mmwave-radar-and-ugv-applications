#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/foreach.hpp>
#include <iostream>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include "std_msgs/Int16MultiArray.h"
#include <unsupported/Eigen/MatrixFunctions>
#include <math.h>

using namespace std;
typedef pcl::PointCloud<pcl::PointXYZI> PointCloud;
//Eigen::Matrix <double, Eigen::Dynamic, 5>  pt;
double h;
int w;
float thres_max = 0.3;
float thres_min = 0.05;
int flag_counter;
int flag;
std_msgs::Int16MultiArray dat;



//void callback(const PointCloud::ConstPtr& msg)
//{
//    printf ("Cloud: width = %d, height = %d\n", msg->width, msg->height);
//    BOOST_FOREACH (const pcl::PointXYZI& pt, msg->points)
//    printf ("\t(%f, %f, %f, %f)\n", pt.x, pt.y, pt.z,pt.intensity);
//}

void callback(const PointCloud::ConstPtr& msg)
{
   PointCloud pcd_msg;
   pcd_msg = *msg;
   w = pcd_msg.width; 
   cout <<  "number of detected points = " << w <<endl;
   w = double(pcd_msg.width);
   float* dx = new float[w]; 
   flag_counter = 0;
   
   for (int i = 0; i <= w-1; i++ )
       {
               dx[i] = sqrt(pcd_msg.points[i].x*pcd_msg.points[i].x+pcd_msg.points[i].y*pcd_msg.points[i].y+pcd_msg.points[i].z*pcd_msg.points[i].z);
	       cout << "object" << i+1 << "=" << pcd_msg.points[i].x << "," << pcd_msg.points[i].y << "," << pcd_msg.points[i].z << "," << pcd_msg.points[i].intensity <<  endl ;
               cout << "dis" << i+1 << "=" << dx[i] << endl;
       } 
   for (int i = 0; i <= w-1; i++)
      {
	       if (dx[i] <= thres_max && dx[i] >=thres_min) {
		       flag_counter += 1;
		       
	       }  
	       
       }
   if (flag_counter >= 1) {
	   flag = 1;
   }
   else {
	   flag = 0;
   }   
   cout << "flag=" << flag << endl;
   return; 

}

int main(int argc, char** argv)
     {
       ros::init(argc, argv, "sub_pcl");
       ros::NodeHandle nh;
       ros::Subscriber sub = nh.subscribe<PointCloud>("/ti_mmwave/radar_scan_pcl_0", 1000, callback);
       ros::Publisher flag_pub = nh.advertise<std_msgs::Int16MultiArray>("obstacle_flag",10);
       ros::Rate rate(20.0);
       while (ros::ok())
       {   
	       dat.data.clear();
	       dat.data.push_back(flag);

	       flag_pub.publish(dat);
	      
	       ros::spinOnce();
	       rate.sleep();
       }       
     }

