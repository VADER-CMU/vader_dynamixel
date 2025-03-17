#!/usr/bin/env python3
import rospy
from vader_msgs.msg import CutterCommand, GripperCommand
from driver import DynamixelDriver, FakeDynamixelDriver
import yaml
import os
import numpy as np
import time

#!/usr/bin/env python
def load_dynamixel_config():
    config_path = os.path.join(os.path.dirname(__file__), '../config/dynamixel_config.yaml')
    with open(config_path, 'r') as file:
        config = yaml.safe_load(file)
    return config

gripperDriver = None
cutterDriver = None
fakeCutter = True
fakeGripper = False

def gripperTransferFunction(percentage):
    gripperMotor1Angle = (percentage/100) * (np.pi/180) * 100
    gripperMotor2Angle = (percentage/100) * (np.pi/180) * 100
    gripperMotor3Angle = (percentage/100) * (np.pi/180) * 100
    return [gripperMotor1Angle, gripperMotor2Angle, gripperMotor3Angle]

def gripperCallback(data):
    rospy.loginfo(rospy.get_caller_id() + " I heard gripper %s", data)
    joint_targets = gripperTransferFunction(data.open_pct)
    try:
        if not gripperDriver.torque_enabled():
            gripperDriver.set_torque_mode(True)
            rospy.loginfo("Torque enabled")
            time.sleep(0.1)
        gripperDriver.set_joints(joint_targets)
    except Exception as e:
        rospy.logerr(f"Failed to set joint positions: {e}")

def cutterTransferFunction(percentage):
    MIN_ANGLE = 228.52
    MAX_ANGLE = 342.77
    return ((percentage / 100) * (MAX_ANGLE - MIN_ANGLE) + MIN_ANGLE) * (np.pi / 180)# rad

def cutterCallback(data):
    rospy.loginfo(rospy.get_caller_id() + " I heard cutter %s", data)
    # Check and set torque enabled
    # Set joint targets according to gripper command percentage
    # TODO optionally disable torque
    joint_targets = [cutterTransferFunction(data.open_pct)]
    try:
        if not cutterDriver.torque_enabled():
            cutterDriver.set_torque_mode(True)
            rospy.loginfo("Torque enabled")
            time.sleep(0.1)
        # print(joint_targets)
        cutterDriver.set_joints(joint_targets)
        
        # time.sleep(0.5)
        # cutterDriver.set_torque_mode(False)
    except Exception as e:
        rospy.logerr(f"Failed to set joint positions: {e}")

def main():
    global gripperDriver, cutterDriver
    rospy.init_node('dynamixel_node', anonymous=True)
    # Load config
    config = load_dynamixel_config()

    # Initialize gripper and cutter drivers
    try:
        if fakeGripper:
            gripperDriver = FakeDynamixelDriver(ids=config['gripper']['ids'])
        else:
            gripperDriver = DynamixelDriver(ids=config['gripper']['ids'], port=config['gripper']['u2d2_usb'],\
                                            baudrate=config['gripper']['u2d2_baudrate'])
        if fakeCutter:
            cutterDriver = FakeDynamixelDriver(ids=config['cutter']['ids'])
        else:
            cutterDriver = DynamixelDriver(ids=config['cutter']['ids'], port=config['cutter']['u2d2_usb'],\
                                            baudrate=config['cutter']['u2d2_baudrate'])
    except Exception as e:
        rospy.logerr(f"Failed to initialize Dynamixel drivers: {e}")
        exit(1)
    
    cutterDriver.set_torque_mode(True)
    gripperDriver.set_torque_mode(True)
    rospy.loginfo("Torque enabled")
    rospy.Subscriber("gripper_command", GripperCommand, gripperCallback)
    rospy.Subscriber("cutter_command", CutterCommand, cutterCallback)
    rospy.loginfo("Dynamixel node initialized successfully")
    # cutterCallback(CutterCommand(open_pct=0))
    gripperCallback(GripperCommand(open_pct=0))
    rospy.spin()
    # Clean up
    gripperDriver.close()
    cutterDriver.close()
    rospy.loginfo("Drivers closed, node is shutting down")
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass