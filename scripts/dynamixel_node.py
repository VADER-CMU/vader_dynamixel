#!/usr/bin/env python3
import rospy
from vader_msgs.msg import CutterCommand, GripperCommand
from driver import DynamixelDriver, FakeDynamixelDriver
import yaml
import os
import numpy as np
import time
from std_msgs.msg import Float64

#!/usr/bin/env python
def load_dynamixel_config():
    config_path = os.path.join(os.path.dirname(__file__), '../config/dynamixel_config.yaml')
    with open(config_path, 'r') as file:
        config = yaml.safe_load(file)
    return config

gripperDriver = None
cutterDriver = None
fakeCutter = False
fakeGripper = False
config = None

def gripperTransferFunction(open_percent):
    if fakeGripper:
        finger_open = 0
        finger_closed = 1.
        return [
            (open_percent/100) *  ((finger_open - finger_closed)) + finger_closed,
            (open_percent/100) *  ((finger_open - finger_closed)) + finger_closed,
            (open_percent/100) *  ((finger_open - finger_closed)) + finger_closed,
        ]
    else:
        GRIPPER1_CLOSED = 270
        GRIPPER1_OPEN = 30
        GRIPPER2_CLOSED = 270
        GRIPPER2_OPEN = 30
        GRIPPER4_CLOSED = 270
        GRIPPER4_OPEN = 30
        return [
            (np.pi/180) *  ((open_percent/100) *  (GRIPPER1_OPEN - GRIPPER1_CLOSED) + GRIPPER1_CLOSED),
            (np.pi/180) *  ((open_percent/100) * (GRIPPER2_OPEN - GRIPPER2_CLOSED) + GRIPPER2_CLOSED),
            (np.pi/180) *  ((open_percent/100) * (GRIPPER4_OPEN - GRIPPER4_CLOSED) + GRIPPER4_CLOSED),
        ]

def gripperCallback(data):
    rospy.loginfo(rospy.get_caller_id() + " I heard gripper %s", data)
    joint_targets = gripperTransferFunction(data.open_pct)
    try:
        if not gripperDriver.torque_enabled():
            gripperDriver.set_torque_mode(True)
            rospy.loginfo("Torque enabled")
            time.sleep(0.1)
        print(joint_targets)
        gripperDriver.set_joints(joint_targets)
        if fakeGripper:
            for topic in config['gripper']['topics']:
                pub = rospy.Publisher(topic, Float64, queue_size=10)
                pub.publish(joint_targets[0])
                print('publishing', joint_targets[0], 'to', topic)
    except Exception as e:
        rospy.logerr(f"Failed to set joint positions: {e}")

def cutterTransferFunction(open_percent):
    if fakeCutter:
        cutter_open = 0
        cutter_closed = -0.75
        return (open_percent / 100) * (cutter_open - cutter_closed) + cutter_closed
    else:    
        MIN_ANGLE = 342.77
        MAX_ANGLE = 205.05
        return ((open_percent / 100) * (MAX_ANGLE - MIN_ANGLE) + MIN_ANGLE) * (np.pi / 180)# rad

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
        if fakeCutter:
            for topic in config['cutter']['topics']:
                pub = rospy.Publisher(topic, Float64, queue_size=10)
                pub.publish(joint_targets[0])
                print('publishing', joint_targets[0], 'to', topic)
        
        # time.sleep(0.5)
        # cutterDriver.set_torque_mode(False)
    except Exception as e:
        rospy.logerr(f"Failed to set joint positions: {e}")

def main():
    global gripperDriver, cutterDriver, config
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
    # cutterCallback(CutterCommand(open_pct=100))

    gripperCallback(GripperCommand(open_pct=100))
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