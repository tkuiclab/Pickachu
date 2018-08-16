//<!-- 2018.06.30 -->
// example to fetch pcd and image data every 20 seconds
//<!-- <author email="wenchih_tai@solomon.com.tw">Wenchih Tai</author> -->

#include <ros/ros.h>  
#include <string.h>
#include "std_msgs/String.h"
#include "accupick3d/cmd.h"
#include <thread>

int main (int argc, char **argv)  
{  
  std::string frameid="world";
  ros::init (argc, argv,frameid);  
  ros::NodeHandle nh("~");  

  std::string subcommandname="/accupick3d/cmd";
 //std::string subcmdStringname="/accupick3d/cmdString";

  ros::Publisher pub = nh.advertise<const accupick3d::cmd> (subcommandname, 10);  
  ros::Rate loop_rateScan(0.5);  // 2 seconds update once
  ros::Rate loop_rate(0.05);  // 20 seconds update once
  int index=0;
  while (ros::ok())  
  {      
    accupick3d::cmd msg;
    //msg.cmd=254;// start scan
    //pub.publish(msg);
    //ros::spinOnce();  
    //std::cout << "StartScan " << index << " " << subcommandname <<" " << msg.cmd <<std::endl;
    //loop_rateScan.sleep();
   //		std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    msg.cmd=255;// get image and pcd data
    pub.publish(msg);
    ros::spinOnce(); 
    std::cout << "Update " << index << " " <<subcommandname <<" " << msg.cmd <<std::endl;
    loop_rate.sleep(); 
    index++;
  }  
    
  return 0;  
}  