#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/foreach.hpp>
#include <iostream>
#include <fstream>
#include <algorithm>
#include <functional>
#include <iterator>
#include <numeric>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Int16MultiArray.h"
#include <unsupported/Eigen/MatrixFunctions>

using namespace std;
typedef pcl::PointCloud<pcl::PointXYZI> PointCloud;
Eigen::Matrix <float, Eigen::Dynamic, 4>  pt;
double w,h;
ofstream p;
PointCloud pcd_msg;
int seq = 0;
int counter = 0;
const int samplesize = 11;
int trigger=0;
int valip;
float suminti[samplesize];
float sumx[samplesize];
float sumy[samplesize];
float sumz[samplesize];
float sx,sy,sz,si,maxi,mini,stdi,stdx,stdy,stdz,meani,meanx,meany,meanz,ps,pk,pw,pflag;
std_msgs::Int16MultiArray dat;




//void callback(const PointCloud::ConstPtr& msg)
//{
//    printf ("Cloud: width = %d, height = %d\n", msg->width, msg->height);
//    BOOST_FOREACH (const pcl::PointXYZI& pt, msg->points)
//    printf ("\t(%f, %f, %f, %f)\n", pt.x, pt.y, pt.z,pt.intensity);
//}

void callback(const PointCloud::ConstPtr& msg)
{
   
   pcd_msg = *msg;
   w = pcd_msg.width; 
   
   w = double(pcd_msg.width);
   
   //for (int i = 0; i <= w-1; i++ )
      // {
        //    cout << "object" << i+1 << "=" << pcd_msg.points[i] << endl ;
      // } 
   return;   
}

int main(int argc, char** argv)
{
       ros::init(argc, argv, "gest_pcl");
       ros::NodeHandle nh;
       ros::Subscriber gest_sub = nh.subscribe<PointCloud>("/ti_mmwave/radar_scan_pcl_0", 100, callback);
       ros::Publisher trigger_pub = nh.advertise<std_msgs::Int16MultiArray>("trigger_flag",10);
       ros::Rate rate(20.0);
       while(ros::ok())
       {
          
          
          si = 0;
          sx = 0;
          sy = 0;
          sz = 0;                    
                         
              //cout <<  "number of detected points = " << w <<endl;
          for (int j = 0; j <= w-1; j++ )
                {
	           if (pcd_msg.points[j].x <= 0.5){		
                        sx = sx + pcd_msg.points[j].x;
                        sy = sy + pcd_msg.points[j].y;
                        sz = sz + pcd_msg.points[j].z;
                        si = si + pcd_msg.points[j].intensity;
                     }		   

                }
          suminti[seq] = si;
          sumx[seq] = sx;
          sumy[seq] = sy;
          sumz[seq] = sz;
          //cout << seq << endl;
          //cout << suminti[seq] << endl;
          //cout << " " << endl;
          seq = seq + 1;    
          if (seq == samplesize) {
              meani = accumulate(begin(suminti),end(suminti),0)/float(seq);
              meanx = accumulate(begin(sumx),end(sumx),0)/float(seq);
              meany = accumulate(begin(sumy),end(sumy),0)/float(seq);
              meanz = accumulate(begin(sumz),end(sumz),0)/float(seq);
              stdi = 0;
              stdx = 0;
              stdy = 0;
              stdz = 0;
              for (int i = 0; i<seq; i++){
                   stdi = stdi + (suminti[i] - meani)*(suminti[i] - meani);
                   stdx = stdx + (sumx[i] - meanx)*(sumx[i] - meanx);
                   stdy = stdy + (sumy[i] - meany)*(sumy[i] - meany);
                   stdz = stdz + (sumz[i] - meanz)*(sumz[i] - meanz);   
              }
              stdi = sqrt(stdi);
              stdx = sqrt(stdx);
              stdy = sqrt(stdy);
              stdz = sqrt(stdz); 
              maxi = *max_element(begin(suminti),end(suminti));              
              mini = *min_element(begin(suminti),end(suminti));
              ps = abs(stdi-0.27)*0.35 + abs(maxi- mini - 0.70) * 0.35 + abs(stdx - 0.022) * 0.1 + abs(stdy - 0.061) *0.1 + abs(stdz-0.0015)*0.1; 
              pw = abs(stdi-13.6541)*0.35 + abs(maxi- mini - 38.9282) * 0.35 + abs(stdx - 0.2253) * 0.1 + abs(stdy - 0.2013) *0.1 + abs(stdz-0.2411)*0.1;
              pk = abs(stdi-22.3647)*0.35 + abs(maxi- mini - 61.7784) * 0.35 + abs(stdx - 0.3064) * 0.1 + abs(stdy - 0.4021) *0.1 + abs(stdz - 0.2556)*0.1;
              pflag =   pk/(pk+pw+ps);
              //cout << stdi << endl;
              //cout << maxi-mini <<endl;
              //cout << stdx <<endl;
              //cout << stdy <<endl;
              //cout << stdz <<endl; 
              //cout << " " << endl;
              cout << "Ps=" << ps <<endl;
              cout << "Pw=" << pw <<endl;
              cout << "Pk=" << pk <<endl;
              cout << "Pflag" << pflag << endl;
              
              if (pflag <= 0.25 && pw-pk>10 && counter >13 ){
                     trigger = 1;
                     cout << trigger << endl;
                     cout << counter <<endl;
                     
              }

              
              cout << " " << endl;
              fill(begin(suminti),end(suminti),0);
              fill(begin(sumx),end(sumx),0);
              fill(begin(sumy),end(sumy),0);
              fill(begin(sumz),end(sumz),0);
            
              seq = 0;
          }    
          counter = counter + 1;
	  dat.data.clear();
          dat.data.push_back(trigger);
          trigger_pub.publish(dat);
           
          
          ros::spinOnce();
          rate.sleep();
       }
}


