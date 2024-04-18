#!/usr/bin/env python3

import os
import cv2
import rospy
import rosbag
import tf2_ros
import tf_conversions
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
import numpy as np

def create_transformation_matrix(translation, rotation):
    quaternion = (rotation.x, rotation.y, rotation.z, rotation.w)
    R = tf_conversions.transformations.quaternion_matrix(quaternion)[:3, :3]
    
    t = np.array([translation.x, translation.y, translation.z])
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = t
    return T

bag_file = 'rosbag/undist_data.bag'
data_save_dir = 'data'

interested_topics = [
    "/locobot/camera/aligned_depth_to_color/camera_info",
    "/locobot/camera/aligned_depth_to_color/image_raw",
    "/locobot/camera/color/camera_info",
    "/locobot/camera/color/image_raw",
    "/tf",
    "/tf_static"
]

for directory in ['color', 'depth', 'pose']:
    os.makedirs(os.path.join(data_save_dir, directory), exist_ok=True)

tf_buffer = tf2_ros.Buffer()
bridge = CvBridge()
image_count = 0  

with rosbag.Bag(bag_file, 'r') as bag:
    for topic, msg, t in bag.read_messages(topics=interested_topics):
        # print(t)
        if topic == "/tf":
            for transform in msg.transforms:
                tf_buffer.set_transform(transform, "default_authority")

        elif topic == "/tf_static":
            for transform in msg.transforms:
                tf_buffer.set_transform_static(transform, "default_authority")
        
        elif topic == "/locobot/camera/color/image_raw":
            color_img = bridge.imgmsg_to_cv2(msg, 'bgr8')
            image_path = os.path.join(data_save_dir, 'color', f"{image_count}.png")
            cv2.imwrite(image_path, color_img)
        
        elif topic == "/locobot/camera/aligned_depth_to_color/image_raw":
            depth_img = bridge.imgmsg_to_cv2(msg, 'passthrough')
            image_path = os.path.join(data_save_dir, 'depth', f"{image_count}.png")
            cv2.imwrite(image_path, depth_img)
            
            try:
                transform_stamped = tf_buffer.lookup_transform('map', 'locobot/camera_color_optical_frame', rospy.Time(0))
                T = create_transformation_matrix(transform_stamped.transform.translation, transform_stamped.transform.rotation)
                np.savetxt(os.path.join(data_save_dir, 'pose', f"{image_count}.txt"), T)
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                np.savetxt(os.path.join(data_save_dir, 'pose', f"{image_count}.txt"), np.zeros((4,4)))  
            image_count += 1
        print(image_count)
    

