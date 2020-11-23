#!/usr/bin/env python
import rospy
from std_msgs.msg import UInt16
from std_msgs.msg import Float64
from std_msgs.msg import String
from sensor_msgs.msg import Imu
from sensor_msgs.msg import MagneticField

class DirectionDetecter(object):
    def __init__(self):
        self.a_x = None
        self.a_y = None
        self.a_z = None
        self.prev_a_x = None
        self.prev_a_y = None
        self.prev_a_z = None

        self.m_x = None
        self.m_y = None
        self.m_z = None
        self.stop_m_x = None
        self.stop_m_y = None
        self.stop_m_z = None
        self.forward_m_x = None
        self.forward_m_y = None
        self.forward_m_z = None        
        self.back_m_x = None
        self.back_m_y = None
        self.back_m_z = None
        self.right_m_x = None
        self.right_m_y = None
        self.right_m_z = None
        self.left_m_x = None
        self.left_m_y = None
        self.left_m_z = None

        self.command = 'stop'
        rospy.Subscriber('imu/data_raw', Imu, self._callback_imu, queue_size=1)
        rospy.Subscriber('imu/mag', MagneticField, self._callback_mag, queue_size=1)
        #self.com_pub = rospy.Publisher('jet_command', String, queue_size=1)
        self.mortor = rospy.Publisher('/jetbot_motors/cmd_str', String, queue_size=1)

    def _callback_imu(self, data):
        self.a_x = data.linear_acceleration.x
        self.a_y = data.linear_acceleration.y
        self.a_z = data.linear_acceleration.z

    def _callback_mag(self, data):
        self.m_x = data.magnetic_field.x*1E6
        self.m_y = data.magnetic_field.y*1E6
        self.m_z = data.magnetic_field.z*1E6

    def mag_init(self):
        while True:
            key = raw_input('stop (y/n): ')
            if key == 'y':
                print('mx:{} my:{} mz:{}'.format(self.m_x, self.m_y, self.m_z))
                self.stop_m_x = self.m_x
                self.stop_m_y = self.m_y
                self.stop_m_z = self.m_z                
                break

        while True:
            key = raw_input('forward  (y/n): ')
            if key == 'y':
                print('mx:{} my:{} mz:{}'.format(self.m_x, self.m_y, self.m_z))
                self.forward_m_x = self.m_x
                self.forward_m_y = self.m_y
                self.forward_m_z = self.m_z                
                break

        while True:          
            key = raw_input('back  (y/n): ')
            if key == 'y':
                print('mx:{} my:{} mz:{}'.format(self.m_x, self.m_y, self.m_z))
                self.back_m_x = self.m_x
                self.back_m_y = self.m_y
                self.back_m_z = self.m_z  
                break

        while True:
            key = raw_input('right  (y/n): ')
            if key == 'y':
                print('mx:{} my:{} mz:{}'.format(self.m_x, self.m_y, self.m_z))
                self.right_m_x = self.m_x
                self.right_m_y = self.m_y
                self.right_m_z = self.m_z
                break

        while True:         
            key = raw_input('left  (y/n): ')
            if key == 'y':
                print('mx:{} my:{} mz:{}'.format(self.m_x, self.m_y, self.m_z))
                self.left_m_x = self.m_x
                self.left_m_y = self.m_y
                self.left_m_z = self.m_z                   
                break

    def mag_update(self):
        print('mx:{} my:{} mz:{}'.format(self.m_x, self.m_y, self.m_z))
        #print('ax:{} ay:{} az:{}'.format(self.a_x, self.a_y, self.a_z))
        mm = 10
        if self.m_x == None or self.m_y == None or self.m_z == None:
            self.command = 'stop'
        else:
            if abs(self.forward_m_x - self.m_x) < mm and abs(self.forward_m_y - self.m_y) < mm and abs(self.forward_m_z - self.m_z) < mm:
                self.command = 'forward'
            elif abs(self.back_m_x - self.m_x) < mm and abs(self.back_m_y - self.m_y) < mm and abs(self.back_m_z - self.m_z) < mm:
                self.command = 'backward' 
            elif abs(self.right_m_x - self.m_x) < mm and abs(self.right_m_y - self.m_y) < mm and abs(self.right_m_z - self.m_z) < mm:
                self.command = 'right'
            elif abs(self.left_m_x - self.m_x) < mm and abs(self.left_m_y - self.m_y) < mm and abs(self.left_m_z - self.m_z) < mm:
                self.command = 'left'
            else:
                self.command = 'stop'
        self.mortor.publish(self.command)

    def update(self):
        if self.a_x == None or self.a_y == None or self.a_z == None or self.prev_a_x == None or self.prev_a_y == None or self.prev_a_z == None:
            self.command = 'stop'
        elif self.a_x - self.prev_a_x > 10:
            self.command = 'left'
        elif self.a_y - self.prev_a_y > 10:
            self.command = 'right'
        elif self.a_z - self.prev_a_z > 10:
            self.command = 'left'
        else:
            self.command = 'stop'       
        self.prev_a_x = self.a_x
        self.prev_a_y = self.a_y
        self.prev_a_z = self.a_z
        self.mortor.publish(self.command)

def main():
    rospy.init_node('direction_detect')

    detecter = DirectionDetecter()
    detecter.mag_init()
    r = rospy.Rate(1)
    while not rospy.is_shutdown():
        detecter.mag_update()
        print('command => {}'.format(detecter.command))
        r.sleep()

if __name__ == '__main__':
    main()
