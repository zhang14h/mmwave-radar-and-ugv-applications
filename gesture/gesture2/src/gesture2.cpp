#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/foreach.hpp>
#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
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
string value;
string line;
vector<float> data_line; 
int seq = 0;
int counter = 0;
int downc = 7;
const int samplesize = 11;
const int Knn = 13;
int trigger=0;
int pretrigger = 0;
int valip;
vector<float> neighbour;
const int length = 800;
const int width = 5;
float suminti[samplesize];
float sumx[samplesize];
float sumy[samplesize];
float sumz[samplesize];
float ps[length];
float pw[length];
float pk[length];
float sx,sy,sz,si,maxi,mini,stdi,stdx,stdy,stdz,meani,meanx,meany,meanz,pflag,diss,disk,disw;
int cs,cw,ck;
vector<vector<float>> sds,sdk,sdw;
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
       ifstream frs("/home/minibot1/catkin_ws/src/gesture/gesture2/features/sds.csv");
       ifstream frw("/home/minibot1/catkin_ws/src/gesture/gesture2/features/sdw.csv"); 
       ifstream frk("/home/minibot1/catkin_ws/src/gesture/gesture2/features/sdk.csv");
       
       while (getline(frs,line)){
           data_line.clear(); 
           istringstream readstr(line);
           for (int j = 0; j < width; j++){
                getline(readstr,value,','); 
                data_line.push_back(atof(value.c_str()));
           } 
           sds.push_back(data_line);
       }
        while (getline(frk,line)){
           data_line.clear(); 
           istringstream readstr(line);
           for (int j = 0; j < width; j++){
                getline(readstr,value,','); 
                data_line.push_back(atof(value.c_str()));
           } 
           sdk.push_back(data_line); 
        }
        while (getline(frw,line)){
           data_line.clear(); 
           istringstream readstr(line);
           for (int j = 0; j < width; j++){
                getline(readstr,value,','); 
                data_line.push_back(atof(value.c_str()));
           } 
           sdw.push_back(data_line);  
        }
       while(ros::ok())
       {
          
          
          si = 0;
          sx = 0;
          sy = 0;
          sz = 0;                    
                         
              //cout <<  "number of detected points = " << w <<endl;
          for (int j = 0; j <= w-1; j++ )
                {
	           if (pcd_msg.points[j].x <= 0.5 && pcd_msg.points[j].x > 0.05 ){	
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
              //fetching the sampled data (11 frames)
              for (int i = 0; i < length ; i++){
             //ps[i] = 0; 
             ps[i] = abs(stdi-sds[i][0])*0.35 + abs(maxi- mini - sds[i][1]) * 0.35 + abs(stdx - sds[i][2]) * 0.1 + abs(stdy - sds[i][3]) *0.1 + abs(stdz-sds[i][4])*0.1; 
             pw[i] = abs(stdi-sdw[i][0])*0.35 + abs(maxi- mini - sdw[i][1]) * 0.35 + abs(stdx - sdw[i][2]) * 0.1 + abs(stdy - sdw[i][3]) *0.1 + abs(stdz-sdw[i][4])*0.1;
             pk[i] = abs(stdi-sdk[i][0])*0.35 + abs(maxi- mini - sdk[i][1]) * 0.35 + abs(stdx - sdk[i][2]) * 0.1 + abs(stdy - sdk[i][3]) *0.1 + abs(stdz - sdk[i][4])*0.1;   
              }
              sort (begin(ps),end(ps));
              sort (begin(pk),end(pk));
              sort (begin(pw),end(pw));
              cs = 0;
              cw = 0;
              ck = 0;
              neighbour.clear();
              while (neighbour.size() < Knn){
                    if (ps[cs] <= pw[cw]){
                        if (ps[cs] <= pk[ck]){
                             neighbour.push_back(ps[cs]);
                             cs = cs + 1;
                        }
                        else {
                             neighbour.push_back(pk[ck]);
                             ck = ck + 1;
                        }    
                    }
                    else {
                         if (pw[cw] <= pk[ck]){
                              neighbour.push_back(pw[cw]);
                              cw = cw + 1;
                         }
                         else {
                              neighbour.push_back(pk[ck]);
                              ck = ck + 1;
                         }
                    }  
              }

              //diss = count_if(begin(ps),end(ps), [&](float n){return n<=10;});      
              //disk = count_if(begin(pk),end(pk), [&](float n){return n<=10;});
              //disw = count_if(begin(pw),end(pw), [&](float n){return n<=10;});
              
              cout << cs <<" " << cw << " " << ck << endl;
              //pflag =   pk/(pk+pw+ps);
              cout << stdi << endl;
              cout << maxi-mini <<endl;
              cout << stdx <<endl;
              cout << stdy <<endl;
              cout << stdz <<endl; 
              cout << " " << endl;
              if (ck > cw && ck > cs && downc == 7){
                  pretrigger = 1;   
              }  
              if (pretrigger == 1){
                  if  ((ck > cw && ck > cs)||(cw > cs && cw > ck)){
                     counter = counter + 1;
                  }
                  downc = downc - 1;
              }
              if (pretrigger == 1 && downc == 0 && counter <=2 && counter >= 1){
                  trigger = 1;
                  downc = 7;
                  counter = 0; 
                  pretrigger = 0;
              }    
              if (pretrigger == 1 && downc == 0) {
                  downc = 7;
                  counter = 0;
                  pretrigger = 0;
              }
              cout << "pretrigger = " << pretrigger << endl;
              cout << "downc = " << downc << endl;
              cout << "counter = " << counter <<endl;
              cout << "trigger = " << trigger << endl;
              
              //if (pflag <= 0.25 && pw-pk>10 && counter >13 ){
              //       trigger = 1;
              //       cout << trigger << endl;
              //       cout << counter <<endl;
                     
              //}

              
              cout << " " << endl;
              //fill(begin(suminti),end(suminti),0);
              //fill(begin(sumx),end(sumx),0);
              //fill(begin(sumy),end(sumy),0);
              //fill(begin(sumz),end(sumz),0);
            
              seq = 0;  
          }    
          
          
	  dat.data.clear();
          dat.data.push_back(trigger);
          trigger_pub.publish(dat);
           
          
          ros::spinOnce();
          rate.sleep();
       }
}


