#<!-- 2018.06.12 -->
#<!-- <author email="wenchih_tai@solomon.com.tw">Wenchih Tai</author> -->
# 
# www.solomon.com.tw
# www.solomon-3d.com.tw

#Solomon Accupick 3D Scanner control driver
roslaunch accupick3d accupick3d.launch 

#3D Scanner StartScan
rostopic pub /accupick3d/cmd accupick3d/cmd "{cmd: 254}"
#Get Image and 3D Point Cloud from Scanner
rostopic pub /accupick3d/cmd accupick3d/cmd "{cmd: 255}"

#Robot MoveTo DataPos 0
rostopic pub /accupick3d/cmdString std_msgs/String "{data: 'DataPos:10.6598:436.972:275.538:-70.0692:-0.312:-179.4104'}"
#Robot MoveTo DataPos 1
rostopic pub /accupick3d/cmdString std_msgs/String "{data: 'DataPos:10.6598:436.972:285.538:-70.0692:-0.312:-179.4104'}"
#Robot MoveTo HomePos
#(14.24:168.73:496.65:0:0:180)
rostopic pub /accupick3d/cmdString std_msgs/String "{data: 'HomePos:'}"
#Rebot Get Position data
rostopic pub /accupick3d/cmdString std_msgs/String "{data: 'GetPos:'}"
### Feed back ROS topics message /accupick3d/msgString  "Pos:x:y:z:ax:ay:az"
# for example  Pos:100:200:300:180:0:0
rostopic echo /accupick3d/msgString

#Robot I/O Setting pinno low
#rostopic pub /accupick3d/cmdString std_msgs/String "{data: 'SetIO:pinno:0'}" 
#for example set pin 1 low 
rostopic pub /accupick3d/cmdString std_msgs/String "{data: 'SetIO:1:0'}" 
#Robot I/O Setting pinno high
#rostopic pub /accupick3d/cmdString std_msgs/String "{data: 'SetIO:pinno:1'}" 
#for example set pin 1 high
rostopic pub /accupick3d/cmdString std_msgs/String "{data: 'SetIO:1:1'}" 

#Robor I/O Reading pinno  0=low 1=high
#rostopic pub /accupick3d/cmdString std_msgs/String "{data: 'GetIO:kind:pinno'}" I = input , O = output
### Feed back ROS topics message /accupick3d/msgString  "I:pinno:0" or "I:pinno:1" or "O:pinno:0" or "O:pinno:1" 
#for example Get Input Pin 1 
rostopic pub /accupick3d/cmdString std_msgs/String "{data: 'GetIO:I:1'}"
#if input pin 1 low then return "I:1:0", if pin 1 high then return "I:1:1" to /accupick3d/msgString
#for example Get Output Pin 1 
rostopic pub /accupick3d/cmdString std_msgs/String "{data: 'GetIO:O:1'}"
#if pin 1 low then return "O:1:0", if pin 1 high then return "O:1:1" to /accupick3d/msgString

#To save the point cloud 3d data and image to default PCD and PPM file
rostopic pub /accupick3d/cmd accupick3d/cmd "{cmd: 300}"
#To load the point cloud 3d data from default PCD file
rostopic pub /accupick3d/cmd accupick3d/cmd "{cmd: 310}"

#To save the point cloud 3d data to  default PCD file
rostopic pub /accupick3d/cmd accupick3d/cmd "{cmd: 301}"
#To load the point cloud 3d data from default PCD file
rostopic pub /accupick3d/cmd accupick3d/cmd "{cmd: 311}"

#To save the image data to  default PPM file
rostopic pub /accupick3d/cmd accupick3d/cmd "{cmd: 302}"
#To load the image data from default PPM file
rostopic pub /accupick3d/cmd accupick3d/cmd "{cmd: 312}"


