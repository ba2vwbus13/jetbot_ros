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
        self.command = 'stop'
        rospy.Subscriber('imu/data_raw', Imu, self._callback_imu, queue_size=1)
        #self.com_pub = rospy.Publisher('jet_command', String, queue_size=1)
        self.mortor = rospy.Publisher('/jetbot_motors/cmd_str', String, queue_size=1)

    def _callback_imu(self, data):
        self.a_x = data.linear_acceleration.x
        self.a_y = data.linear_acceleration.y
        self.a_z = data.linear_acceleration.z

    def update(self):
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

    r = rospy.Rate(1)
    while not rospy.is_shutdown():
        detecter.update()
        print('command => {}'.format(detecter.command))
        r.sleep()

if __name__ == '__main__':
    main()
