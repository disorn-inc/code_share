#!/usr/bin/env python
import rospy
import operator
from geometry_msgs.msg import Twist
from std_msgs.msg import Int16,Float64,UInt8,Bool
from sensor_msgs.msg import Joy
import numpy as np
from std_msgs.msg import Float64,UInt8,Int64



class JoyManual():
    def __init__(self):
        """initial all parameter"""
        self.sub_joy = rospy.Subscriber('/joy', Joy, self.joycommand, queue_size = 10)
        self.pub_cmd_vel = rospy.Publisher('/Donkey/Arduino/cmd_vel',Twist, queue_size = 1)
        self.pub_brake = rospy.Publisher('/Donkey/Arduino/brake',Bool, queue_size = 1)
        self.pub_emer = rospy.Publisher('/Donkey/Arduino/emergency',Bool, queue_size = 1)
        rospy.on_shutdown(self.fnShutDown)
    def joycommand(self,data):
        twist = Twist()
        joy_data = data.axes
        joy_bot = data.buttons 
        #print(joy_data[4])
        twist.linear.x = joy_data[4]*1
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = joy_data[0]*1
        brake = joy_bot[5]
        emergency = joy_bot[4]
        self.pub_cmd_vel.publish(twist)
        self.pub_brake.publish(brake)
        self.pub_emer.publish(emergency)
    def fnShutDown(self):
        rospy.loginfo("Shutting down.")
        twist = Twist()
        twist.linear.x = 0
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 0
        self.pub_cmd_vel.publish(twist)
    def main(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('joymanual')
    node = JoyManual()
    node.main()