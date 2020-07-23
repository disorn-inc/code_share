#!/usr/bin/env python
import rospy
from std_msgs.msg import Int16,Float64,UInt8,String,Float64MultiArray
import numpy as np
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats
#from sensor_msgs.msg import imu



class ConvertImu():
    def __init__(self):
        self.sub_imu = rospy.Subscriber('/Donkey/Arduino/imu',String,self.get_imu,queue_size = 10)
        self.pub_imu = rospy.Publisher('/imu_convert',numpy_msg(Floats), queue_size = 10)
    def get_imu(self,data):
        imu_string = data.data
        imu = imu_string.split(",")
        imu = imu[1:7]
        imu_array = np.array(imu,dtype=np.float32)
        #imu_array = np.float_(imu) #another version
        #imu = float(imu)
        imu_ac = ((imu_array[0:3])* 1/16384. * 9.80665)
        #imu_ac = ((imu_array[0:3])* 1/16384.)
        imu_w = ((imu_array[3:6])/131)
        offset_a = np.array([-0.5,0.3,8])
        offset_w = np.array([-3.,2.,-1.])
        # imu_ac = imu_ac - offset_a
        # imu_w = imu_w - offset_w
        imu_w_deg = imu_w * 180 / 3.1415926
        imu_sum = np.append(imu_ac,imu_w)
        imu_list = imu_sum.tolist()
        self.pub_imu.publish(imu_sum)
        print(imu_sum)
    def fnShutDown(self):
        rospy.loginfo("Shutting down.")
    def main(self):
        rospy.spin()






if __name__ == '__main__':
    rospy.init_node('convert_imu')
    node = ConvertImu()
    node.main()
