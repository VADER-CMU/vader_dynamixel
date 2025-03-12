#!/usr/bin/env python3

import rospy
from vader_msgs.msg import CutterCommand, GripperCommand
import random

def main():
    rospy.init_node('dynamixel_talker_test', anonymous=True)
    pub = rospy.Publisher('cutter_command', CutterCommand, queue_size=10)
    rate = rospy.Rate(0.2)  # once every 20 seconds
    while not rospy.is_shutdown():
        command = CutterCommand()
        command.open_pct = int(random.random() * 100)
        rospy.loginfo(f"Publishing CutterCommand: {command}")
        pub.publish(command)
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass