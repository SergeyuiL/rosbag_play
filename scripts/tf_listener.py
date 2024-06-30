#!/usr/bin/env python3

import rospy
import tf2_ros
import tf_conversions
import argparse
import numpy as np
import rosbag

def create_transformation_matrix(translation, rotation):
    quaternion = (rotation.x, rotation.y, rotation.z, rotation.w)
    R = tf_conversions.transformations.quaternion_matrix(quaternion)[:3, :3]
    
    t = np.array([translation.x, translation.y, translation.z])
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = t
    return T

def listen_to_transforms(bag_file):
    rospy.init_node('tf_listener_node')

    tf_buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tf_buffer)

    with rosbag.Bag(bag_file, 'r') as bag:
        for topic, msg, t in bag.read_messages(topics=['/tf', '/tf_static']):
            if topic == "/tf":
                for transform in msg.transforms:
                    tf_buffer.set_transform(transform, "default_authority")
            elif topic == "/tf_static":
                for transform in msg.transforms:
                    tf_buffer.set_transform_static(transform, "default_authority")

    rate = rospy.Rate(1)  # 1 Hz
    while not rospy.is_shutdown():
        try:
            # 'map', 'locobot/base_footprint'
            
            transform_stamped = tf_buffer.lookup_transform('locobot/base_footprint', 'locobot/camera_color_optical_frame', rospy.Time(0))
            T = create_transformation_matrix(transform_stamped.transform.translation, transform_stamped.transform.rotation)
            print("Transformation Matrix from 'map' to 'base_footprint':\n", T)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            print("Could not retrieve transformation:", e)
        
        rate.sleep()

if __name__ == "__main__":
    # parser = argparse.ArgumentParser(description="Listen to transforms and print the transformation matrix from 'map' to 'base_footprint'.")
    # parser.add_argument('--bag_file', type=str, required=True, help='Path to the ROS bag file.')
    # args = parser.parse_args()
    
    bag_file = "/root/catkin_ws/src/rosbag_play/rosbag/room_1004.bag"

    listen_to_transforms(bag_file)
