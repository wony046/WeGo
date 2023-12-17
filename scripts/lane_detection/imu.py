#!/usr/bin/env python
# -*- coding:utf-8 -*-
#imu 센싱
import rospy
import math
from sensor_msgs.msg import Imu
from std_msgs.msg import String

class limo_imu:
    def __init__(self):
        rospy.init_node('imu', anonymous=True)
        self.linear_acceleration_x = 0
        self.linear_acceleration_z = 0
        self.pitch = 0
        self.start_bump = math.pi / 38
        self.last_bump = -(math.pi / 35)
        self.loop_time = 0
        rospy.Subscriber("/imu", Imu, self.pitch_calculate_callback)
        self.imu_pub = rospy.Publisher("/limo/imu_pitch", String, queue_size=2)
    
    def pitch_calculate_callback(self, data):
        self.linear_acceleration_x = data.linear_acceleration.x
        self.linear_acceleration_z = data.linear_acceleration.z
        if self.linear_acceleration_z != 0:
            self.wait_time = rospy.get_time()
            self.pitch = math.atan(self.linear_acceleration_x / self.linear_acceleration_z)
            if (self.pitch >= self.start_bump or self.pitch <= self.last_bump):
                if self.wait_time - self.loop_time >= 0.1:
                    self.imu_pub.publish("bump")
            else:
                self.loop_time = rospy.get_time()
                self.imu_pub.publish("not_bump")

def run():
    new_class = limo_imu()
    rospy.spin()

if __name__ == '__main__':
    try:
        run()
    except rospy.ROSInterruptException:
        pass

