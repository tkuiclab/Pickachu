#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from std_msgs.msg import Empty
import std_srvs.srv
from accupick3d.msg import cmd

pub = rospy.Publisher('accupick3d/cmd', cmd, queue_size=1)

def ScannerCmd(cmd_code):
  msg = cmd()
  msg.cmd = cmd_code
  print(msg)
  pub.publish(msg)

def TakePicture(req):
  print("Take a picture")
  ScannerCmd(254)
  return std_srvs.srv.EmptyResponse()

def GetPicture(req):
  print("Get a picture")
  ScannerCmd(255)
  return std_srvs.srv.EmptyResponse()

if __name__ == '__main__':
  try:
    rospy.init_node('main', anonymous=True)
    print("PickaChu~ Picka Picka~")

    s0 = rospy.Service('take_picture', std_srvs.srv.Empty, TakePicture)
    s1 = rospy.Service('get_picture', std_srvs.srv.Empty, GetPicture)
    ## 3D Scanner StartScan
    #ScannerCmd(254)
    ## Get Image and 3D Point Cloud from Scanner
    #ScannerCmd(255)

    rospy.spin()

  except rospy.ROSInterruptException:
    pass
