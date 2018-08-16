#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from accupick3d.msg import cmd

def ScannerCmd(cmd_code):
  pub = rospy.Publisher('accupick3d/cmd', cmd, queue_size=1)

  msg = cmd()
  msg.cmd = cmd_code
  print(msg)
  pub.publish(msg)

if __name__ == '__main__':
  try:
    rospy.init_node('main', anonymous=True)
    print("PickaChu~ Picka Picka~")

    ## 3D Scanner StartScan
    ScannerCmd(254)
    ## Get Image and 3D Point Cloud from Scanner
    #ScannerCmd(255)

  except rospy.ROSInterruptException:
    pass
