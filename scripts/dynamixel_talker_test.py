#!/usr/bin/env python3

import rospy
from vader_msgs.msg import CutterCommand, GripperCommand
import random

targets = [0, 20, 40, 60, 80, 100, 80, 60, 40, 20]
def main():
    rospy.init_node('dynamixel_talker_test', anonymous=True)
    pub = rospy.Publisher('cutter_command', CutterCommand, queue_size=10)
    pub2 = rospy.Publisher('gripper_command', GripperCommand, queue_size=10)
    rate = rospy.Rate(0.5)  

    curr_target_idx = 0
    while not rospy.is_shutdown():
        # command = CutterCommand()
        # command.open_pct = int(random.random() * 100)
        # rospy.loginfo(f"Publishing CutterCommand: {command}")
        # pub.publish(command)
        # rate.sleep()
        command = CutterCommand()
        command.open_pct = int(targets[curr_target_idx])
        rospy.loginfo(f"Publishing CutterCommand: {command}")
        pub.publish(command)

        command2 = GripperCommand()
        command2.open_pct = int(targets[curr_target_idx])
        rospy.loginfo(f"Publishing GripperCommand: {command2}")
        pub2.publish(command2)


        curr_target_idx += 1
        if curr_target_idx == len(targets):
            curr_target_idx = 0
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass