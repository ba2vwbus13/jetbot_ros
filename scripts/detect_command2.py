#!/usr/bin/env python
import rospy
from std_msgs.msg import UInt16
from std_msgs.msg import Float64
from std_msgs.msg import String
from sensor_msgs.msg import Imu
from sensor_msgs.msg import MagneticField
import readchar

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
        self.prev_m_x = None
        self.prev_m_y = None
        self.prev_m_z = None

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
        print('mx:{} my:{} mz:{}'.format(self.m_x, self.m_y, self.m_z))
        self.prev_m_x = self.m_x
        self.prev_m_y = self.m_y
        self.prev_m_z = self.m_z

    def mag_update(self):
        print('mx:{} my:{} mz:{}'.format(self.m_x, self.m_y, self.m_z))
        #print('ax:{} ay:{} az:{}'.format(self.a_x, self.a_y, self.a_z))
        if self.m_x == None or self.m_y == None or self.m_z == None or self.prev_m_x == None or self.prev_m_y == None or self.prev_m_z == None:
            self.command = 'stop'
        else:
            #dx = abs(self.m_x - self.prev_m_x)
            #dy = abs(self.m_y - self.prev_m_y)
            #dz = abs(self.m_z - self.prev_m_z)
            dx = self.m_x - self.prev_m_x
            dy = self.m_y - self.prev_m_y
            dz = self.m_z - self.prev_m_z
            print('dx:{} dy:{} dz:{}'.format(dx, dy, dz))
            mm = 2
            mx = 25
            if abs(dx) < mm and abs(dy) > mx:
                self.command = 'forward'
            elif abs(dx) < mm and abs(dz) > mx:
                self.command = 'back'
            elif dx < 0 and abs(dy) > mx and abs(dz) < mm:
                self.command = 'right'
            elif dx > 0 and abs(dy) > mx and abs(dz) < mm:
                self.command = 'left'
            else:
                self.command = 'stop'

        self.mortor.publish(self.command)

    def update(self):
        #print('mx:{} my:{} mz:{}'.format(self.m_x, self.m_y, self.m_z))
        #print('ax:{} ay:{} az:{}'.format(self.a_x, self.a_y, self.a_z))
        if self.a_x == None or self.a_y == None or self.a_z == None or self.prev_a_x == None or self.prev_a_y == None or self.prev_a_z == None:
            self.command = 'stop'
        elif self.a_x - self.prev_a_x > 10:
            self.command = 'left'
            #print('{} : {}'.format(self.command, self.a_x - self.prev_a_x))
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
    while True:
        key = raw_input('Init ? (y/n): ')
        if key == 'y':
            detecter.mag_init()
        else:
            break
    
    while not rospy.is_shutdown():
        detecter.mag_update()
        print('command => {}'.format(detecter.command))
        r.sleep()

if __name__ == '__main__':
    main()
