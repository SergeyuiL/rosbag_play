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
import shutil

parser = argparse.ArgumentParser(description="Process and save data from ROS bag files.")
parser.add_argument('--bag_file', type=str, required=True, help='Path to the ROS bag file.')
parser.add_argument('--data_save_dir', type=str, required=True, help='Directory to save the data.')

args = parser.parse_args()

def clear_dir(data_dir):
    if not os.path.exists(data_dir):
        print(f"The directory {data_dir} does not exist.")
        return
    
    for filename in os.listdir(data_dir):
        file_path = os.path.join(data_dir, filename)
        try:
            if os.path.isfile(file_path) or os.path.islink(file_path):
                os.unlink(file_path)  
            elif os.path.isdir(file_path):
                shutil.rmtree(file_path)  
        except Exception as e:
            print(f"Failed to delete {file_path}. Reason: {e}")

def create_transformation_matrix(translation, rotation):
    quaternion = (rotation.x, rotation.y, rotation.z, rotation.w)
    R = tf_conversions.transformations.quaternion_matrix(quaternion)[:3, :3]
    
    t = np.array([translation.x, translation.y, translation.z])
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = t
    return T

def convert_to_pose(T):
    x, y = T[0, 3], T[1, 3]
    yaw = np.arctan2(T[1, 0], T[0, 0])  # Extract yaw from rotation matrix
    return x, y, yaw

def calculate_distance(pose1, pose2):
    return np.sqrt((pose2[0] - pose1[0])**2 + (pose2[1] - pose1[1])**2)

bag_file = args.bag_file
data_save_dir = args.data_save_dir
# bag_file = "/root/catkin_ws/src/rosbag_play/rosbag/room_1004.bag"
# data_save_dir = "/root/catkin_ws/src/rosbag_play/data/room_1004"

clear_dir(data_save_dir)

interested_topics = [
    "/locobot/camera/aligned_depth_to_color/camera_info",
    "/locobot/camera/aligned_depth_to_color/image_raw",
    "/locobot/camera/color/image_raw",
    "/tf",
    "/tf_static"
]

for directory in ['rgb', 'depth']:
    os.makedirs(os.path.join(data_save_dir, directory), exist_ok=True)

tf_buffer = tf2_ros.Buffer()
bridge = CvBridge()
image_count = 0  

camera_info_saved = False
last_pose = None
interval_trans = 0.2  # meter
interval_yaw = 0.1  # rad
pose_file = os.path.join(data_save_dir, "node_pose.txt")

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
            image_path = os.path.join(data_save_dir, 'rgb', f"rgb_{image_count}.png")
            cv2.imwrite(image_path, color_img)
        
        elif topic == "/locobot/camera/aligned_depth_to_color/image_raw":
            depth_img = bridge.imgmsg_to_cv2(msg, 'passthrough')
            image_path = os.path.join(data_save_dir, 'depth', f"depth_{image_count}.png")
            cv2.imwrite(image_path, depth_img)
            
            try:
                # 'locobot/camera_color_optical_frame'
                transform_stamped = tf_buffer.lookup_transform('map', 'locobot/base_footprint', rospy.Time(0))
                T = create_transformation_matrix(transform_stamped.transform.translation, transform_stamped.transform.rotation)
                current_pose = convert_to_pose(T)
                if current_pose is not None:
                    if last_pose is None:
                        with open(pose_file, 'a') as f:
                            f.write(f"{image_count} {current_pose[0]:.4f} {current_pose[1]:.4f} {current_pose[2]:.4f}\n")
                        last_pose = current_pose
                        print(image_count)
                        image_count += 1
                    else:
                        distance = calculate_distance(last_pose, current_pose)
                        if distance > interval_trans or np.abs(current_pose[2] - last_pose[2]) > interval_yaw:
                            with open(pose_file, 'a') as f:
                                f.write(f"{image_count} {current_pose[0]:.4f} {current_pose[1]:.4f} {current_pose[2]:.4f}\n")
                            print(image_count)
                            image_count += 1   
                            last_pose = current_pose
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                print(e)  
            
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

os.remove(os.path.join(data_save_dir, 'rgb', f"rgb_{image_count}.png"))    
os.remove(os.path.join(data_save_dir, 'depth', f"depth_{image_count}.png"))  
    

