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
import argparse
import json

parser = argparse.ArgumentParser(description="Process and save data from ROS bag files.")
parser.add_argument('--bag_file', type=str, required=True, help='Path to the ROS bag file.')
parser.add_argument('--data_save_dir', type=str, required=True, help='Directory to save the data.')

args = parser.parse_args()

def create_transformation_matrix(translation, rotation):
    quaternion = (rotation.x, rotation.y, rotation.z, rotation.w)
    R = tf_conversions.transformations.quaternion_matrix(quaternion)[:3, :3]
    
    t = np.array([translation.x, translation.y, translation.z])
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = t
    return T

bag_file = args.bag_file
data_save_dir = args.data_save_dir

interested_topics = [
    "/locobot/camera/aligned_depth_to_color/camera_info",
    "/locobot/camera/aligned_depth_to_color/image_raw",
    "/locobot/camera/color/image_raw",
    "/tf",
    "/tf_static"
]

for directory in ['color', 'depth', 'pose']:
    os.makedirs(os.path.join(data_save_dir, directory), exist_ok=True)

tf_buffer = tf2_ros.Buffer()
bridge = CvBridge()
image_count = 0  

camera_info_saved = False

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
            image_path = os.path.join(data_save_dir, 'color', f"rgb_{image_count}.png")
            cv2.imwrite(image_path, color_img)
        
        elif topic == "/locobot/camera/aligned_depth_to_color/image_raw":
            depth_img = bridge.imgmsg_to_cv2(msg, 'passthrough')
            image_path = os.path.join(data_save_dir, 'depth', f"depth_{image_count}.png")
            cv2.imwrite(image_path, depth_img)
            
            try:
                transform_stamped = tf_buffer.lookup_transform('map', 'locobot/camera_color_optical_frame', rospy.Time(0))
                T = create_transformation_matrix(transform_stamped.transform.translation, transform_stamped.transform.rotation)
                np.savetxt(os.path.join(data_save_dir, 'pose', f"{image_count}.txt"), T)
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                np.savetxt(os.path.join(data_save_dir, 'pose', f"{image_count}.txt"), np.zeros((4,4)))  
            image_count += 1
            
        elif topic == "/locobot/camera/aligned_depth_to_color/camera_info" and not camera_info_saved:
            camera_info = {
                'header': {
                    'seq': msg.header.seq,
                    'stamp': str(msg.header.stamp),  
                    'frame_id': msg.header.frame_id
                },
                'height': msg.height,
                'width': msg.width,
                'distortion_model': msg.distortion_model,
                'D': msg.D,
                'K': msg.K,
                'R': msg.R,
                'P': msg.P,
                'binning_x': msg.binning_x,
                'binning_y': msg.binning_y,
                'roi': {
                    'x_offset': msg.roi.x_offset,
                    'y_offset': msg.roi.y_offset,
                    'height': msg.roi.height,
                    'width': msg.roi.width,
                    'do_rectify': msg.roi.do_rectify
                }
            }
            with open(os.path.join(data_save_dir, 'camera_info.json'), 'w') as f:
                json.dump(camera_info, f, indent=4)
            camera_info_saved = True  
        print(image_count)
    

