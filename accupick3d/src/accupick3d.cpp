//<!-- 2018.07.09 -->
//<!-- <author email="wenchih_tai@solomon.com.tw">Wenchih Tai</author> -->


#include "accupick3d.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <chrono>

void SaveImageAsPPM( const sensor_msgs::Image& msg, const char* filename )
{
  int channel=3;
  int magicno=3;
  if (msg.encoding == "bgr8" )
  {
    channel=3;
    magicno=3;
    std::cout << filename <<"\n" << msg.encoding << " Channel :"<<channel<<std::endl;
  }
  else if(msg.encoding == "mono8")
  {
    channel=1;
    magicno=2;
    std::cout << filename <<"\n" << msg.encoding << " Channel :"<<channel<<std::endl;
  }
  else
  {
    std::cout << filename <<"\n" << msg.encoding << " Format Error!"<<std::endl;
    return;  // Can only handle the rgb8 encoding
  }
  FILE* file = fopen( filename, "w" );
  if(file==NULL)
  {
    std::cout << filename << "Open Error!"<<std::endl;
    return;
  }
  fprintf( file, "P%i\n",magicno);
  fprintf( file, "%i %i\n", msg.width, msg.height );
  fprintf( file, "255\n" );
  for ( uint32_t y = 0; y < msg.height; y++ )
  {
    for ( uint32_t x = 0; x < msg.width; x++ )
    {
      // Get indices for the pixel components
	uint32_t ByteIdx = y*msg.step + channel*x;
	if(channel==3)
	{
          uint32_t blueByteIdx = ByteIdx;
          uint32_t greenByteIdx = ByteIdx + 1;
          uint32_t redByteIdx = ByteIdx + 2;
          fprintf( file, "%i %i %i ", 
          msg.data[ redByteIdx ], 
          msg.data[ greenByteIdx ], 
          msg.data[ blueByteIdx ] );
	}
	else if(channel==1)
	{
          fprintf( file, "%i ",msg.data[ ByteIdx ]);
	}
    }
    fprintf( file, "\n" );
  }
  fclose( file );
  std::cout << filename << " Saved!"<<std::endl;
}

void LoadPPMImage(sensor_msgs::Image& msg, const char* filename )
{
  FILE* file = fopen( filename, "r" );
  if(file==NULL)
  {
    std::cout << filename << "Open Error!"<<std::endl;
    return;
  }
  int w,h;
  int channel=3;
  int magicno=3;
  fscanf( file, "P%i\n" ,&magicno);
  fscanf( file, "%i %i\n", &w, &h );
  fscanf( file, "255\n" );
  if(magicno==3)
  {
    msg.encoding = "bgr8";
    channel=3;
  }
  else if(magicno==2)
  {
    msg.encoding = "mono8";
    channel=1;
  }
  else
  {
    fclose(file);
    return;
  }
  std::cout << filename <<"\n" << msg.encoding << " Channel :"<<channel<<std::endl;
  msg.width=w;
  msg.height=h;
  msg.step=w*channel;
  msg.data.resize(msg.step*msg.height);
  int r,g,b;
  for ( uint32_t y = 0; y < msg.height; y++ )
  {
    for ( uint32_t x = 0; x < msg.width; x++ )
    {
      // Get indices for the pixel components
      uint32_t ByteIdx = y*msg.step + channel*x;     
      if(channel==3)
      {        
        uint32_t blueByteIdx = ByteIdx;
        uint32_t greenByteIdx = ByteIdx + 1;
        uint32_t redByteIdx = ByteIdx + 2;
        fscanf( file, "%i %i %i ", &r,&g,&b);
        msg.data[ redByteIdx ]=r;
        msg.data[ greenByteIdx ]=g; 
        msg.data[ blueByteIdx ]=b;
      }
      else if(channel==1)
      {
        fscanf( file, "%i ", &r);
        msg.data[ ByteIdx ]=r;      
      }
    }
    fscanf( file, "\n" );
  }
  fclose( file );
  std::cout << filename << " Load finished!"<<std::endl;
}


Solomon_AccuPick3D::Solomon_AccuPick3D() 
  :map()
//  comm_request_slot(&Solomon_AccuPick3D::requestHandle, *this),
  //slot_debug(&Solomon_driver::rosDebug, *this)
{
}

void Solomon_AccuPick3D::create(int port)
{
  portno=port;
  comm_request_sig.connect(std::string("/comm_request_cloud"));
  img_request_sig.connect(std::string("/comm_request_image"));
  // 2018.06.07 append to read back the message
  message_return_sig.connect(std::string("/message_return"));
  //sig_debug.connect(std::string("/ros_debug"));
  connect_thread= std::thread(&Solomon_AccuPick3D::spin,this);
}

void Solomon_AccuPick3D::spin()
{  
  ROS_INFO("spin start...");
	std::string str="hello";
	int rtn = 0;
	while(true)
	{
		socket_open();
		socket_listen();
		ROS_INFO("tcp communication listen");

		while(getConnectStatus())
		{
			rtn=socket_read();
		}
//		std::this_thread::sleep_for(std::chrono::milliseconds(1000));
	}
	ROS_INFO("spin end...");
}

void Solomon_AccuPick3D::socket_open()
{
	int enable = 1;
	bzero((char *)&serv_addr,sizeof(serv_addr));
	sockfd = 0;
	ROS_INFO("open socket");
	sockfd = socket(AF_INET, SOCK_STREAM, 0);
	if(sockfd<0)
	{
	  ROS_ERROR("Error open socket ");
	}
	else
	{
	  ROS_INFO("socket create success ID =%i",sockfd);
	}
	bConnected = false;

	ROS_INFO("setsockopt");
	if (setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR, &enable, sizeof(int)) < 0)
	  ROS_ERROR("setsockopt(SO_REUSEADDR) failed");
	bzero((char *) &serv_addr, sizeof(serv_addr));
	serv_addr.sin_family = AF_INET;
	serv_addr.sin_addr.s_addr = INADDR_ANY;
	serv_addr.sin_port = htons(portno);
}


void Solomon_AccuPick3D::socket_listen()
{
  socklen_t clilen;
  int enable = 1;
  ROS_INFO("bind");
  if (bind(sockfd, (struct sockaddr *) &serv_addr,sizeof(serv_addr)) < 0)
    ROS_ERROR("ERROR on binding");
  else
  {
    ROS_INFO("listen");
    listen(sockfd,5);
    clilen = sizeof(cli_addr);
    ROS_INFO("accept");
    newsockfd = accept(sockfd,(struct sockaddr *) &cli_addr,&clilen);
    if (newsockfd < 0)
      ROS_ERROR("ERROR on accept");
    else
      bConnected=true;
  }
}

long Solomon_AccuPick3D::readbyte(long rno,char* ptr)
{
  char* buffer=ptr;
  //2018.07.09 modify to reduce the image buffer
  long leftno=rno;
  long n;
  long index=0;
  while(leftno>0)
  {
    n = recv(newsockfd,buffer+index,leftno,0);
    leftno-=n;
    index+=n;
  }
  return index;
}


bool Solomon_AccuPick3D::readint(long& no)
{
  no=0;
  //return recv(newsockfd,(char*)&no,4,0);
  if(readbyte(4,(char*)&no)==4)
    return true;
  else
    return false;
}

bool Solomon_AccuPick3D::readOnefloat(float* ff)
{
  if(readbyte(4,(char*)ff)==4L)
    return true;
  else
    return false;
}


long Solomon_AccuPick3D::readfloat(long nf,float* fs)
{
  long i=0;
  for(i=0;i<nf;i++)
  {
    if(!readOnefloat(fs+i))
      break;
  }
  return i;
}


int getInt(unsigned char h,unsigned char l)
{
  return (((int)h)*256+(int)l);
}

long Solomon_AccuPick3D::readPoint1UV(long np)
{
  int u,v;
  int index=0;
  int indexmap=0;
  int w=Img.width;
  int w1=w;
  int h=Img.height;
  int channel;
  if(Img.encoding=="mono8")
    channel=1;
  else
    channel=3;
  long mapsize=(w1 * h);
  long size1 = mapsize *channel;

  map.resize(mapsize);
	
  char* bgr=(char*)&(Img.data[0]);
  unsigned char* c;
  pcl::PointXYZRGB p1;


  //int perf=4;
  long nf=0;

  float fs[4];  
  long i=0;
  for(i=0;i<np;i++)
  {	
    nf=readfloat(4,fs);
    if(nf!=4L)
      break;
    p1.x=fs[0];
    p1.y=fs[1];
    p1.z=fs[2];
    c=(unsigned char*)(fs+3);
   
    u= getInt(*(c+1),*(c+0));
    v= getInt(*(c+3),*(c+2));	
    indexmap=v*w1+u;
    index=indexmap*channel;
    if(index>=0 && index<size1)
    {
	if(channel==1)
	{
          p1.b=*(bgr+index);
	  p1.g=*(bgr+index);
	  p1.r=*(bgr+index);
	}
	else
	{
	  p1.b=*(bgr+index);
	  p1.g=*(bgr+index+1);
	  p1.r=*(bgr+index+2);
	}
    }
    if(indexmap>=0 && indexmap<mapsize)
	map[indexmap]=p1;
  }
  return i;
}

long Solomon_AccuPick3D::readPointUV(long np)
{
  return readPoint1UV(np);
}

int Solomon_AccuPick3D::readPointCloudUV()
{
  long no;
  long n=readint(no);
  if (n < 0)
  {
    ROS_ERROR("ERROR reading from socket");
    socket_close();
  }
  else if(n == 0)
  {
    ROS_ERROR("read data = 0");
    socket_close();
  }
  else
  {
    long np=no/4/4;
    ROS_INFO("Message head: %ld bytes, %ld point3d", no,np);
    long retnp=readPointUV(np);
    ROS_INFO("POINTXYZUV read finished: %ld / %ld",retnp,np);
  }
  return 1;
}
/*
std::string
mat_type2encoding(int mat_type)
{
  switch (mat_type) {
    case CV_8UC1:
      return "mono8";
    case CV_8UC3:
      return "bgr8";
    case CV_16SC1:
      return "mono16";
    case CV_8UC4:
      return "rgba8";
    default:
      throw std::runtime_error("Unsupported encoding type");
  }
}
*/

int Solomon_AccuPick3D::readImage()
{
  long w,h,channel;
  long n=0;
  n=readint(w);
  n=readint(h);
  n=readint(channel);
  Img.width=w;
  Img.height=h;
  int w1=w;
  if(channel==1)
    Img.encoding = "mono8";
  else
    Img.encoding = "bgr8";
  Img.step = w1*channel;
  long size = w1 * h *channel;
  ROS_INFO("Image size: %ld",size);
  Img.data.resize(size);
  long ret=readbyte(size,(char*)&(Img.data[0]));
  ROS_INFO("Image size: W=%ld H=%ld Channel=%ld Read=%ld / %ld",w,h,channel,ret,size);
  return 1;
}

int Solomon_AccuPick3D::readString(std::string& msg)
{
  unsigned char ll;
  long n=readbyte(1,(char*)&ll); // read string length, the maximum is 255 bytes
  //std::cout<< "Readd StringLen:"<< ll << std::endl;
  long len=ll;
  char ss[len+1];
  bzero(ss,len+1);
  readbyte(len,(char*)ss);
  ss[len]=0;
  msg=ss;
  //std::cout<< "Readd StringLen:"<< len << std::endl;
  ROS_INFO("Read String:%s",msg.c_str());
  return 1;
}

int Solomon_AccuPick3D::RunCmd(enum cmdType cc)
{
	int ret=1;
		switch(cc)
		{
			/*
			case cmdReadCloud:
        cloud.clear();
			  readPointCloud();
				comm_request_sig.emit(cloud);
			  break;
			case cmdReadImage:
			  readImage();
				img_request_sig.emit(Img);
			  break;
			case cmdReadCloudImage:
        cloud.clear();
			  readPointCloud();
				readImage();
				img_request_sig.emit(Img);
				comm_request_sig.emit(cloud);
			  break;
      */
		 //---- append 2018.06.30 -----
		  case cmdReadString:
			{
        readString(msgstr);
				std_msgs::String mm;
				mm.data=msgstr;
				message_return_sig.emit(mm);
			}
			  break;
      //------------------------------
			case cmdReadImageCloudUV:
			  readImage();
				img_request_sig.emit(Img);
        //cloud.clear();
				map.clear();
        readPointCloudUV();
				//comm_request_sig.emit(cloud);
				comm_request_sig.emit(map);
			  break;
			case cmdShowImageCloud:
			  img_request_sig.emit(Img);
				//comm_request_sig.emit(cloud);
				comm_request_sig.emit(map);
				break;
			case cmdShowCloud:
			  //comm_request_sig.emit(cloud);
				comm_request_sig.emit(map);
				break;
			case cmdShowImage:
			  img_request_sig.emit(Img);
				break;
			case cmdSaveAllDefault:
			  std::cout << "Save PCD "<<pcd_file;
			  //pcl::io::savePCDFile(pcd_file,cloud);			
				pcl::io::savePCDFile(pcd_file,map);	
				std::cout << "Save finished! "<<std::endl;
				std::cout << "Save Image "<<img_file;
			  SaveImageAsPPM(Img,img_file.c_str());
				std::cout << "Save finished! "<<std::endl;
         break;
			case cmdSaveCloudDefault:
			  std::cout << "Save PCD "<<pcd_file<<std::endl;
			  //pcl::io::savePCDFile(pcd_file,cloud);			
				pcl::io::savePCDFile(pcd_file,map);	
				std::cout << "Save finished! "<<pcd_file<<std::endl;
			  break;
			case cmdSaveImageDefault:
			  std::cout << "Save Image "<<img_file<<std::endl;
			  SaveImageAsPPM(Img,img_file.c_str());
			  break;
			case cmdLoadAllDefault:
			std::cout << "Load PCD "<<pcd_file<<std::endl;
			  //pcl::io::loadPCDFile(pcd_file,cloud);				
				//comm_request_sig.emit(cloud);
				pcl::io::loadPCDFile(pcd_file,map);				
				comm_request_sig.emit(map);
				std::cout << "Load finished! "<<pcd_file<<std::endl;
				std::cout << "Load Image "<<img_file<<std::endl;
        LoadPPMImage(Img,img_file.c_str());	
				img_request_sig.emit(Img);
			  break;
			case cmdLoadCloudDefault:
			  std::cout << "Load PCD "<<pcd_file<<std::endl;
				 //pcl::io::loadPCDFile(pcd_file,cloud);				
				//comm_request_sig.emit(cloud);
			  pcl::io::loadPCDFile(pcd_file,map);				
				comm_request_sig.emit(map);
				std::cout << "Load finished! "<<pcd_file<<std::endl;
			  break;
			case cmdLoadImageDefault:
			  std::cout << "Load Image "<<img_file<<std::endl;
        LoadPPMImage(Img,img_file.c_str());	
				img_request_sig.emit(Img);	  
			  break;
		}
	return ret;
}

int Solomon_AccuPick3D::socket_read()
{
	long cmd;
	long n=readint(cmd);// to read the command ID 4 bytes first from socket
	if (n < 0)
	{
		ROS_ERROR("cmd ERROR reading from socket");
		socket_close();
	}
	else if(n == 0)
	{
		ROS_ERROR("cmd read data = 0");
		socket_close();
	}
	else
	{ 
//		std::cout<<"Cmd:"<<cmd<<std::endl;
    enum cmdType cc=(enum cmdType)cmd;
		RunCmd(cc);
	}
	return n;


}

void Solomon_AccuPick3D::socket_write(int cmd)
{
//	write(sockfd,buf,);
	ROS_INFO("socket_write cmd:%d",cmd);
	write(newsockfd, &cmd, sizeof(int));
}

void Solomon_AccuPick3D::socket_write(std::string buf)
{
	int ll=buf.length();
	if(ll<=0)
	{
	  ROS_ERROR("Socket_write error ! Empty string\n");
	  return;
	}
	int cmd=cmdSendString;
	char bufbyte[ll+6];
	memcpy(bufbyte,&cmd,4);// write the cmdSendString ID first
	bufbyte[4]=(ll&0xFF); // append the string length information
	memcpy(bufbyte+5,buf.c_str(),ll); // append the string data information
	ROS_INFO("socket_write\n%s\n",buf.c_str());
	write(newsockfd, bufbyte, ll+5);
}

void Solomon_AccuPick3D::socket_close()
{
  bConnected=false;
  close(sockfd);
}

bool Solomon_AccuPick3D::getConnectStatus()
{
  return bConnected;
}
//reserve
void Solomon_AccuPick3D::socket_connect(std::string host)
{

//connect socket
	//init
	int rtn=0;
	socket_open();
		//pri_serv_addr_.sin_addr = "10.1.2.81";

	server_ = gethostbyname(host.c_str());

	bcopy((char *)server_->h_addr,(char *)&pri_serv_addr_.sin_addr.s_addr,server_->h_length);
	pri_serv_addr_.sin_port=portno;
	rtn=connect(sockfd,(struct sockaddr *)&pri_serv_addr_,sizeof(pri_serv_addr_));
	if(rtn<0)
		ROS_ERROR("socket connect fail");
	else
		bConnected=true;

}

Solomon_AccuPick3D::~Solomon_AccuPick3D() 
{
  // TODO Auto-generated destructor stub
  socket_close();
}
