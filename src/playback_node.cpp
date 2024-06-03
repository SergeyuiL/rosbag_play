#include <chrono>
#include <functional>
#include <iostream>
#include <memory>
#include <string>
#include <fstream>
#include <filesystem>
#include <map>
#include <queue>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/serialization.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_msgs/msg/tf_message.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"
#include "rosbag2_cpp/reader.hpp"
#include "rosbag2_cpp/readers/sequential_reader.hpp"

using namespace std::chrono_literals;
namespace fs = std::filesystem;

class PlaybackNode : public rclcpp::Node
{
public:
    PlaybackNode(const std::string & bag_filename, const std::string & data_dir)
    : Node("playback_node"), data_directory_(data_dir)
    {
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        timer_ = this->create_wall_timer(100ms, std::bind(&PlaybackNode::timer_callback, this));
        reader_.open(bag_filename);

        fs::create_directories(data_directory_ + "/depth");
        fs::create_directories(data_directory_ + "/color");
        fs::create_directories(data_directory_ + "/pose");
    }

private:
    struct ImageData {
        cv::Mat image;
        rclcpp::Time timestamp;
    };

    std::map<int, ImageData> depth_images;
    std::map<int, ImageData> color_images;
    std::map<int, geometry_msgs::msg::TransformStamped> poses;
    int image_count_ = 0;

    void timer_callback()
    {
        while (reader_.has_next()) {
            auto msg = reader_.read_next();

            if (msg->topic_name == "/realsense2/aligned_depth_to_color/image_raw") {
                store_image(msg, "/depth");
            } else if (msg->topic_name == "/camera/color/image_raw") {
                store_image(msg, "/color");
            }
            else if (msg->topic_name == "/tf" || msg->topic_name == "/tf_static") {
                process_tf_message(msg);
            }

            check_and_save_data();
        }
    }

    void store_image(const rosbag2_storage::SerializedBagMessageSharedPtr& msg, const std::string& folder)
    {
        auto image_msg = std::make_shared<sensor_msgs::msg::Image>();
        rclcpp::SerializedMessage serialized_msg(*msg->serialized_data);
        serialization_.deserialize_message(&serialized_msg, image_msg.get());

        cv::Mat image = cv_bridge::toCvCopy(image_msg, image_msg->encoding)->image;
        if (folder == "/depth") {
            depth_images[image_count_].image = image;
            depth_images[image_count_].timestamp = image_msg->header.stamp;
        } else if (folder == "/color") {
            color_images[image_count_].image = image;
            color_images[image_count_].timestamp = image_msg->header.stamp;
        }
    }

    void process_tf_message(const rosbag2_storage::SerializedBagMessageSharedPtr& msg)
    {
        auto tf_msg = std::make_shared<tf2_msgs::msg::TFMessage>();
        rclcpp::SerializedMessage serialized_msg(*msg->serialized_data);
        serialization_.deserialize_message(&serialized_msg, tf_msg.get());

        for (auto& transform : tf_msg->transforms) {
            if (transform.header.frame_id == "map" && transform.child_frame_id == "realsense2_color_optical_frame") {
                poses[image_count_] = transform;
            }
        }
    }

    void check_and_save_data() {
        if (depth_images.find(image_count_) != depth_images.end() && color_images.find(image_count_) != color_images.end() && poses.find(image_count_) != poses.end()) {
            // Save images and pose
            cv::imwrite(data_directory_ + "/depth/" + std::to_string(image_count_) + ".png", depth_images[image_count_].image);
            cv::imwrite(data_directory_ + "/color/" + std::to_string(image_count_) + ".png", color_images[image_count_].image);
            save_pose(poses[image_count_]);

            // Remove the used data to prevent memory overflow
            depth_images.erase(image_count_);
            color_images.erase(image_count_);
            poses.erase(image_count_);
            image_count_++;
        }
    }

    void save_pose(const geometry_msgs::msg::TransformStamped& transform)
    {
        std::ofstream file(data_directory_ + "/pose/" + std::to_string(image_count_) + ".txt");
        if (file.is_open()) {
            file << "Translation: (" << transform.transform.translation.x << ", " << transform.transform.translation.y << ", " << transform.transform.translation.z << ")\n";
            file << "Rotation: (" << transform.transform.rotation.x << ", " << transform.transform.rotation.y << ", " << transform.transform.rotation.z << ", " << transform.transform.rotation.w << ")\n";
            file.close();
        }
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Serialization<sensor_msgs::msg::Image> serialization_;
    rclcpp::Serialization<tf2_msgs::msg::TFMessage> tf_serialization_;
    rosbag2_cpp::readers::SequentialReader reader_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::string data_directory_;
};

int main(int argc, char ** argv)
{
    if (argc != 3) {
        std::cerr << "Usage: " << argv[0] << " <bag> <data_directory>" << std::endl;
        return 1;
    }

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PlaybackNode>(argv[1], argv[2]));
    rclcpp::shutdown();

    return 0;
}
