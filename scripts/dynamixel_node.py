#!/usr/bin/env python3
import rospy
from vader_msgs.msg import CutterCommand, GripperCommand
from driver import DynamixelDriver, FakeDynamixelDriver
import yaml
import os

#!/usr/bin/env python
def load_dynamixel_config():
    config_path = os.path.join(os.path.dirname(__file__), '../config/dynamixel_config.yaml')
    with open(config_path, 'r') as file:
        config = yaml.safe_load(file)
    return config

gripperDriver = None
cutterDriver = None
fakeDrivers = True

def gripperCallback(data):
    rospy.loginfo(rospy.get_caller_id() + " I heard %s", data)
    # Check and set torque enabled
    # Set joint targets according to gripper command percentage
    # TODO optionally disable torque

def cutterCallback(data):
    rospy.loginfo(rospy.get_caller_id() + " I heard %s", data)
    # Check and set torque enabled
    # Set joint targets according to gripper command percentage
    # TODO optionally disable torque

def main():
    global gripperDriver, cutterDriver
    rospy.init_node('dynamixel_node', anonymous=True)
    # Load config
    config = load_dynamixel_config()

    # Initialize gripper and cutter drivers
    try:
        if fakeDrivers:
            gripperDriver = FakeDynamixelDriver(ids=config['gripper']['ids'])
            cutterDriver = FakeDynamixelDriver(ids=config['cutter']['ids'])
        else:
            gripperDriver = DynamixelDriver(ids=config['gripper']['ids'], port=config['gripper']['u2d2_usb'],\
                                            baudrate=config['gripper']['u2d2_baudrate'])

            cutterDriver = DynamixelDriver(ids=config['cutter']['ids'], port=config['cutter']['u2d2_usb'],\
                                            baudrate=config['cutter']['u2d2_baudrate'])
    except Exception as e:
        rospy.logerr(f"Failed to initialize Dynamixel drivers: {e}")
        exit(1)

    rospy.Subscriber("gripper_command", GripperCommand, gripperCallback)
    rospy.Subscriber("cutter_command", CutterCommand, cutterCallback)
    rospy.loginfo("Dynamixel node initialized successfully")
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass