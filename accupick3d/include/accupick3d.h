//<!-- 2018.07.09 -->
//<!-- <author email="wenchih_tai@solomon.com.tw">Wenchih Tai</author> -->

#ifndef SOURCE_DIRECTORY__SOLOMON_ACCUPICK3D_SRC_ACCUPICK3D_H_
#define SOURCE_DIRECTORY__SOLOMON_ACCUPICK3D_SRC_ACCUPICK3D_H_

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <netdb.h>
#include <stdio.h>

#include <thread>

#include <ecl/sigslots.hpp>

#include <pcl/point_cloud.h>  
#include <pcl_conversions/pcl_conversions.h>  
#include <sensor_msgs/PointCloud2.h>  
//#include <pcl/io/pcd_io.h>

#include <sensor_msgs/image_encodings.h>


enum cmdType:long
{
	cmdReadImageCloudUV=4,
	//-------communication to ROSPAthPlanning.cs ----------------------------
	cmdReadString=252, // append after 2018.06.07 version, the ID to read the string from Robot
	cmdSendString=253, // request to send the string command to Robot
	cmdStartScan=254, // request the 3D scanner to start scan from ROS
	cmdTramitteImageCloud=255, // request to send the image and point cloud 3d from ROS
	//------------ User define command --------------------------------------
    // from 256 to 300 is the data show communication insterface between ROS and MoveIT
	cmdShowImageCloud=256,
	cmdShowCloud=257,
	cmdShowImage=258,
    // from 300 to 309 is the data save communication insterface between ROS and file
	cmdSaveAllDefault=300,
	cmdSaveCloudDefault=301,	
	cmdSaveImageDefault=302,
	// from 300 to 309 is the data load communication insterface between ROS and file
	cmdLoadAllDefault=310,
	cmdLoadCloudDefault=311,
	cmdLoadImageDefault=312
};

class Solomon_AccuPick3D {
private:
	int sockfd,newsockfd;
	int portno = 1048; 
	struct sockaddr_in pri_serv_addr_,serv_addr,cli_addr;
	struct hostent *server_;
	//int flag,bufSize;
	bool bConnected;
	std::thread connect_thread;

	ecl::Signal<const pcl::PointCloud<pcl::PointXYZRGB>&> comm_request_sig;
	ecl::Signal<const sensor_msgs::Image&> img_request_sig;
  	ecl::Signal<const std_msgs::String&> message_return_sig;

public:
	Solomon_AccuPick3D();
	virtual ~Solomon_AccuPick3D();
	bool getConnectStatus();
	void create(int port=1048);

    void socket_open();
	void socket_connect(std::string host);
	void socket_listen();
	int socket_read();
	void socket_write(int cmd);
	void socket_write(std::string buf);
	void socket_close();

	void spin();


    int GetMessage(std::string& message);

    long readbyte(long rno,char* ptr);
    bool readint(long& no);
    bool readOnefloat(float* ff);
    long readfloat(long nf,float* fs);
    long readPoint1UV(long np);
    long readPointUV(long np);

	int readPointCloudUV();
	int readImage();
    int readString(std::string& msg);

	//pcl::PointCloud<pcl::PointXYZRGB> cloud;
	sensor_msgs::Image Img;
	std::string pcd_file="sample.pcd";
	std::string img_file="sample.ppm";
	std::string msgstr;

    pcl::PointCloud<pcl::PointXYZRGB> map;

	int RunCmd(enum cmdType cc);
    // retrive the point cloud point from the (u,v) image coordinate
    bool GetMapPointXYZRGB(int u,int v,pcl::PointXYZRGB& p)
	{
		bool ret=false;
		int w=Img.width;
		int h=Img.height;
		int index=w*v+u;
		if(index>=0 && index<map.size())
		{
		  p=map[index];
		  ret=true;
		}	
		return ret;
	}

};

#endif 
