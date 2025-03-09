# import rospy
# from std_msgs.msg import String
import yaml
import os

#!/usr/bin/env python
def load_dynamixel_config():
    config_path = os.path.join(os.path.dirname(__file__), 'dynamixel_config.yaml')
    with open(config_path, 'r') as file:
        config = yaml.safe_load(file)
    return config

config = load_dynamixel_config()
print(config)

# def talker():
#     pub = rospy.Publisher('test_topic', String, queue_size=10)
#     rospy.init_node('dynamixel_node', anonymous=True)
#     rate = rospy.Rate(10) # 10hz
#     while not rospy.is_shutdown():
#         hello_str = "Hello ROS %s" % rospy.get_time()
#         rospy.loginfo(hello_str)
#         pub.publish(hello_str)
#         rate.sleep()

# if __name__ == '__main__':
#     try:
#         talker()
#     except rospy.ROSInterruptException:
#         pass