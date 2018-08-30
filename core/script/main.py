#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from std_msgs.msg import Empty
import std_srvs.srv
#from accupick3d.msg import cmd

def ScannerCmd(cmd_code):
  pub = rospy.Publisher('accupick3d/cmd', cmd, queue_size=1)

  msg = cmd()
  msg.cmd = cmd_code
  print(msg)
  pub.publish(msg)

def TakePicture(req):
  print("Take a picture")
  return std_srvs.srv.EmptyResponse()

if __name__ == '__main__':
  try:
    rospy.init_node('main', anonymous=True)
    print("PickaChu~ Picka Picka~")

    s = rospy.Service('take_picture', std_srvs.srv.Empty, TakePicture)
    ## 3D Scanner StartScan
    #ScannerCmd(254)
    ## Get Image and 3D Point Cloud from Scanner
    #ScannerCmd(255)

    rospy.spin()

  except rospy.ROSInterruptException:
    pass
