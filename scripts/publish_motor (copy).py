#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
 
def run():
 
    motor = rospy.Publisher('/jetbot_motors/cmd_str', String, queue_size=10)
    rospy.init_node('run', anonymous=True)
 
    print('run:forward')
    motor.publish("forward")
    rospy.sleep(3.)
 
    print('run:backward')
    motor.publish("backward")
    rospy.sleep(3.)
 
    print('run:left')
    motor.publish("left")
    rospy.sleep(3.)
 
    print('run:right')
    motor.publish("right")
    rospy.sleep(3.)
 
    print('run:stop')
    motor.publish("stop")
 
if __name__ == '__main__':
    run()


