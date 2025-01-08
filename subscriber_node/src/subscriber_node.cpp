#include <cstdio>
#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/byte_multi_array.hpp>

#include "node_options/cli_options.hpp"

static
node_options::Options
parse_options(int argc, char ** argv)
{
  auto non_ros_args = rclcpp::remove_ros_arguments(argc, argv);
  std::vector<char *> non_ros_args_c_strings;
  for (auto & arg : non_ros_args) {
    non_ros_args_c_strings.push_back(&arg.front());
  }
  int non_ros_argc = static_cast<int>(non_ros_args_c_strings.size());
  auto options = node_options::Options(non_ros_argc, non_ros_args_c_strings.data());

  return options;
}

std::string byteArrayToString(const std::vector<uint8_t>& data) {
  return std::string(data.begin(), data.end());
}


class Subscriber : public rclcpp::Node
{
public:
    explicit Subscriber(const node_options::Options & options)
    : Node(options.node_name)
    {
        auto callback =
          [this](const std_msgs::msg::ByteMultiArray byte_msg) -> void
          {
            RCLCPP_INFO(this->get_logger(), "%s", byteArrayToString(byte_msg.data));
          };
        
        rclcpp::QoS qos(rclcpp::KeepLast(10));
        sub_ = create_subscription<std_msgs::msg::ByteMultiArray>(
            options.topic_name, qos, callback);
    }

private:
    rclcpp::Subscription<std_msgs::msg::ByteMultiArray>::SharedPtr sub_;
};

int main(int argc, char * argv[])
{
  auto options = parse_options(argc, argv);

  // クライアントライブラリの初期化
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);
  rclcpp::init(argc, argv);

  auto node = std::make_shared<Subscriber>(options);
  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}