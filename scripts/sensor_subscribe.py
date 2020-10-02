#!/usr/bin/env python
import rospy
from std_msgs.msg import UInt16
from std_msgs.msg import Float64
from sensor_msgs.msg import Imu
from sensor_msgs.msg import MagneticField

def recv_buzzer(data):
    rospy.loginfo(type(data))
    rospy.loginfo(data.data)

def _callback_imu2(data):
    rospy.loginfo(type(data))
    rospy.loginfo(data.linear_acceleration.x)

def _callback_imu(data):
    print('------------------------------------------')
    print('acc(x):{}'.format(data.linear_acceleration.x))
    print('acc(y):{}'.format(data.linear_acceleration.y))
    print('acc(z):{}'.format(data.linear_acceleration.z))
    #print('ori(x):{}'.format(data.orientation.x))
    #print('ori(y):{}'.format(data.orientation.y))
    #print('ori(z):{}'.format(data.orientation.z))
    #print('ori(w):{}'.format(data.orientation.w))
    print('ang(x):{}'.format(data.angular_velocity.x))
    print('ang(y):{}'.format(data.angular_velocity.y))
    print('ang(z):{}'.format(data.angular_velocity.z))

def _callback_temp(data):
    #rospy.loginfo(type(data))
    print('------------------------------------------')
    print('temp:{}'.format(data.data))

def _callback_mag(data):
    #rospy.loginfo(type(data))
    print('------------------------------------------')
    print('mag(x):{}'.format(data.magnetic_field.x))
    print('mag(y):{}'.format(data.magnetic_field.y))
    print('mag(z):{}'.format(data.magnetic_field.z))
"""         
    self._imu_data_raw = msg

    self._calculate_heading_angle(
        self._imu_data_raw.angular_velocity.z,
        self._imu_data_raw.header.stamp
    )

    self._filter_acceleration(self.linear_acceleration) """

if __name__ == '__main__':
    rospy.init_node('buzzer')
    rospy.Subscriber('buzzer', UInt16, recv_buzzer)
    rospy.Subscriber('imu/data_raw', Imu, _callback_imu, queue_size=1)
    rospy.Subscriber('imu/mag', MagneticField, _callback_mag, queue_size=1)
    rospy.Subscriber('imu/temperature', Float64, _callback_temp, queue_size=1)
    rospy.spin()