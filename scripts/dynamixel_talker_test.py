#!/usr/bin/env python3

import rospy
from vader_msgs.msg import CutterCommand, GripperCommand


def main():
    rospy.init_node('dynamixel_talker_test', anonymous=True)
    pub = rospy.Publisher('gripper_command', GripperCommand, queue_size=10)
    rate = rospy.Rate(0.5)  # once every 20 seconds
    while not rospy.is_shutdown():
        command = GripperCommand()
        command.open_pct = 1
        rospy.loginfo(f"Publishing GripperCommand: {command}")
        pub.publish(command)
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass