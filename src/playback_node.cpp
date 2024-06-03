#include <chrono>
#include <functional>
#include <iostream>
#include <memory>
#include <string>
#include <fstream>
#include <filesystem>

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
    : Node("playback_node"), data_directory_(data_dir), image_count_(0)
    {
        depth_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/camera/depth/image_raw", 10);
        rgb_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/camera/color/image_raw", 10);
        info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>("/camera/info", 10);
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        timer_ = this->create_wall_timer(100ms, std::bind(&PlaybackNode::timer_callback, this));
        reader_.open(bag_filename);

        fs::create_directories(data_directory_ + "/depth");
        fs::create_directories(data_directory_ + "/color");
        fs::create_directories(data_directory_ + "/pose");
    }

private:
    void timer_callback()
    {
        while (reader_.has_next()) {
            auto msg = reader_.read_next();

            if (msg->topic_name == "/camera/depth/image_raw" || msg->topic_name == "/camera/color/image_raw") {
                publish_image(msg, msg->topic_name);
            }
            else if (msg->topic_name == "/camera/info" && !camera_info_saved_) {
                publish_camera_info(msg);
                camera_info_saved_ = true;
            }
            else if (msg->topic_name == "/tf" || msg->topic_name == "/tf_static") {
                process_tf_message(msg);
            }
        }
    }

    void publish_image(const rosbag2_storage::SerializedBagMessageSharedPtr& msg, const std::string& topic_name)
    {
        auto image_msg = std::make_shared<sensor_msgs::msg::Image>();
        rclcpp::SerializedMessage serialized_msg(*msg->serialized_data);
        serialization_.deserialize_message(&serialized_msg, image_msg.get());

        if (topic_name == "/camera/depth/image_raw") {
            depth_pub_->publish(*image_msg);
        } else if (topic_name == "/camera/color/image_raw") {
            rgb_pub_->publish(*image_msg);
        }

        save_image(image_msg, topic_name);
        image_count_++;
    }

    void save_image(const sensor_msgs::msg::Image::SharedPtr& image_msg, const std::string& topic_name)
    {
        cv::Mat image = cv_bridge::toCvCopy(image_msg, image_msg->encoding)->image;
        std::string file_path = data_directory_ + (topic_name == "/camera/depth/image_raw" ? "/depth/" : "/color/") + std::to_string(image_count_) + ".png";
        cv::imwrite(file_path, image);
    }

    void publish_camera_info(const rosbag2_storage::SerializedBagMessageSharedPtr& msg)
    {
        auto info_msg = std::make_shared<sensor_msgs::msg::CameraInfo>();
        rclcpp::SerializedMessage serialized_msg(*msg->serialized_data);
        serialization_.deserialize_message(&serialized_msg, info_msg.get());
        info_pub_->publish(*info_msg);
        save_camera_info(info_msg);
    }

    void save_camera_info(const sensor_msgs::msg::CameraInfo::SharedPtr& info_msg)
    {
        std::ofstream file(data_directory_ + "/camera_info.json");
        if (file.is_open()) {
            file << "{\n";
            file << "  \"width\": " << info_msg->width << ",\n";
            file << "  \"height\": " << info_msg->height << ",\n";
            file << "  \"distortion_model\": \"" << info_msg->distortion_model << "\",\n";
            file << "  \"D\": [" << join(info_msg->d, ", ") << "],\n";
            file << "  \"K\": [" << join(info_msg->k, ", ") << "],\n";
            file << "  \"R\": [" << join(info_msg->r, ", ") << "],\n";
            file << "  \"P\": [" << join(info_msg->p, ", ") << "]\n";
            file << "}\n";
            file.close();
        }
    }

    template<typename T>
    std::string join(const std::vector<T>& vec, const std::string& delim)
    {
        std::ostringstream oss;
        for (size_t i = 0; i < vec.size(); ++i) {
            if (i != 0) {
                oss << delim;
            }
            oss << vec[i];
        }
        return oss.str();
    }

    void process_tf_message(const rosbag2_storage::SerializedBagMessageSharedPtr& msg)
    {
        auto tf_msg = std::make_shared<tf2_msgs::msg::TFMessage>();
        rclcpp::SerializedMessage serialized_msg(*msg->serialized_data);
        serialization_.deserialize_message(&serialized_msg, tf_msg.get());

        for (auto& transform : tf_msg->transforms) {
            if (transform.header.frame_id == "map" && transform.child_frame_id == "realsense2_color_optical_frame") {
                save_pose(transform);
            }
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
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_pub_, rgb_pub_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr info_pub_;
    rclcpp::Serialization<sensor_msgs::msg::Image> serialization_;
    rclcpp::Serialization<tf2_msgs::msg::TFMessage> tf_serialization_;
    rosbag2_cpp::readers::SequentialReader reader_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::string data_directory_;
    int image_count_;
    bool camera_info_saved_ = false;
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
