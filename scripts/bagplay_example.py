import tf2_py
import rosbag
import rospy
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped

bag_file = 'path_to_bag_file.bag'  # 此处需要指定具体的bag文件路径
interested_topics = [
    "/tf_static",
    "/tf",
    # 其他感兴趣的主题
]

tf_buffer = tf2_py.BufferCore()

with rosbag.Bag(bag_file, 'r') as bag:
    topic: str
    timestamp: rospy.Time
    terminated = False
    for topic, msg, timestamp in bag.read_messages(topics=interested_topics):
        if topic == "/tf":
            for transform in msg.transforms:
                tf_buffer.set_transform(transform, "default_authority")

        elif topic == "/tf_static":
            for transform in msg.transforms:
                tf_buffer.set_transform_static(transform, "default_authority")

        # 需要处理的其他主题和逻辑
