#! /usr/bin/env python


import rosunit
import unittest
import rostest
import rospy
from std_msgs.msg import String
import time
#import sys
PKG = 'rostest'
NAME = 'rotate_robot_test_ros'

class TestRobotControl(unittest.TestCase):
   def setUp(self):

       self.success = False
       self.test_sub = rospy.Subscriber("/chatter", String, self.callback)

   def callback(self, msg):
       print(rospy.get_caller_id(), " responce: %s" % msg.data)
       self.success = True # msg.data

   def test_publish(self):
       
       print(self.success)
       timeout_t = time.time() + 10.0  # 10 seconds
       while not rospy.is_shutdown() and not self.success and time.time() < timeout_t:
           time.sleep(0.1)
           print(self.success)
       self.assert_(self.success)

if __name__ == '__main__':
   rostest.rosrun(PKG, NAME, TestRobotControl)
