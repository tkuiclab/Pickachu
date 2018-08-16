//<!-- 2018.07.09 -->
//<!-- <author email="wenchih_tai@solomon.com.tw">Wenchih Tai</author> -->

#include<ros/ros.h>  
#include<pcl/point_cloud.h>  
#include<pcl_conversions/pcl_conversions.h>  
#include<sensor_msgs/PointCloud2.h>  
#include<pcl/io/pcd_io.h>

#include "accupick3d.h"
#include "accupick3d/cmd.h"
#include <math.h>
#include <string.h>

#include <sensor_msgs/image_encodings.h>


ros::Publisher pcl_pub;
ros::Publisher img_pub;
ros::Publisher msg_pub;
std::string frameid="world";
Solomon_AccuPick3D Accupick3D;

void imageCb(const sensor_msgs::Image& msg)
{
  img_pub.publish(msg);
}

void CmdCallBack(const accupick3d::cmd& msg)
{
  if(msg.cmd<256)
    Accupick3D.socket_write(msg.cmd);
  else
    Accupick3D.RunCmd((enum cmdType)msg.cmd);
}

void CmdStringCallBack(const std_msgs::String& msg)
{
    std::string scmd=msg.data.c_str();
    Accupick3D.socket_write(scmd);
}

//-- append 2018.06.07 ---------------------------
void MsgStringCallBack(const std_msgs::String& msg)
{ 
    msg_pub.publish(msg);   
    std::string smsg=msg.data.c_str();
    std::cout <<"Message Return:" << smsg << std::endl;
}
//-------------------------------------------------

void requestHandle(const pcl::PointCloud<pcl::PointXYZRGB>& cloud)
{
	  sensor_msgs::PointCloud2 output;  	
    pcl::toROSMsg(cloud,output);      
    output.header.stamp=ros::Time::now();
    output.header.frame_id  =frameid;    
    pcl_pub.publish(output);   	
}


int main (int argc, char **argv)  
{  
    int port=1048;
    int demomode=0;
    ROS_INFO("\naccupick3d port:%d\n",port);
    ros::init (argc, argv, "world");  
    ros::NodeHandle nh("~");  
  
    frameid="world";
    std::string advertisename="/accupick3d/output";
    std::string advertiseimage="/accupick3d/image";
    //-- append 2018.06.07 ---------------------------
    std::string advertisemsg="/accupick3d/msgString";
    //-------------------------------------------------

    std::string subscribename="/accupick3d/pcPkg";
    std::string subcommandname="/accupick3d/cmd";
    std::string subcmdStringname="/accupick3d/cmdString";
    

    std::string pcd_file="sample.pcd";
    std::string img_file="sample.ppm";
    int auto_load_data=0;

    nh.param<std::string>("frame_id",frameid,"world");
    nh.param<std::string>("cloud_output_name",advertisename,"/accupick3d/output");
    nh.param<std::string>("image_output_name",advertiseimage,"/accupick3d/image");
    //-- append 2018.06.07 ---------------------------
    nh.param<std::string>("msg_output_name",advertisemsg,"/accupick3d/msgString");
    //-------------------------------------------------

    nh.param<std::string>("cloud_input_name",subscribename,"/accupick3d/pcPkg");
    nh.param<std::string>("command_input_name",subcommandname,"/accupick3d/cmd");
    nh.param<std::string>("command_input_string_name",subcmdStringname,"/accupick3d/cmdString");
    
    nh.param<std::string>("pcd_file",pcd_file,"sample.pcd");
    nh.param<std::string>("img_file",img_file,"sample.ppm");
    nh.param<int>("auto_load_data",auto_load_data,0);

    std::cout << "auto_Load_data = " <<auto_load_data << std::endl;

    ROS_INFO("\naccupick3d node frame_id:%s\n",frameid.c_str());
    ROS_INFO("\naccupick3d node cloud_output_name:%s\n",advertisename.c_str());
    
    ecl::Slot<const pcl::PointCloud<pcl::PointXYZRGB>&> comm_request_slot(&requestHandle);
    comm_request_slot.connect(std::string("/comm_request_cloud"));
    ecl::Slot<const sensor_msgs::Image&> img_request_slot(&imageCb);
    img_request_slot.connect(std::string("/comm_request_image"));

    ROS_INFO("\naccupick3d advertise\n");
    pcl_pub = nh.advertise<sensor_msgs::PointCloud2> (advertisename, 10);  
    img_pub = nh.advertise<sensor_msgs::Image> (advertiseimage, 10);  

    ros::Subscriber sub_cmd = nh.subscribe<const accupick3d::cmd&>(subcommandname,10,CmdCallBack);
    ros::Subscriber sub_cmdString = nh.subscribe<const std_msgs::String&>(subcmdStringname,10,CmdStringCallBack);
    //-- append 2018.06.07 ---------------------------
    ecl::Slot<const std_msgs::String&> message_return_slot(&MsgStringCallBack);
    message_return_slot.connect(std::string("/message_return"));
    msg_pub = nh.advertise<std_msgs::String> (advertisemsg, 10);  
    //------------------------------------------------
    ROS_INFO("\naccupick3d create:%d\n",port);
    Accupick3D.pcd_file=pcd_file;
    Accupick3D.img_file=img_file;
    Accupick3D.create(port);

    if(auto_load_data>0)
    {
      Accupick3D.RunCmd(cmdLoadAllDefault);
    }
    ros::spin();
	  ros::waitForShutdown();
  
    //ros::Rate loop_rate(0.5);  
    //while (ros::ok())  
    //{      
     // ros::spinOnce();  
     // loop_rate.sleep(); 
    //}  
    
    return 0;  
}  
