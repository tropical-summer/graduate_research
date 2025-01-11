#include <cstdio>
#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/byte_multi_array.hpp>

#include "node_options/cli_options.hpp"
#include "publisher_node/msg/performance_header.hpp"
#include "publisher_node/msg/int_message.hpp"

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

class Subscriber : public rclcpp::Node
{
public:
  explicit Subscriber(const node_options::Options & options)
    : Node(options.node_name)
  {
    // 複数のトピック名を扱う場合
    for (size_t i = 0; i < options.topic_names.size(); ++i) {
      const std::string & topic_name = options.topic_names[i];

      auto callback = [this, topic_name](const publisher_node::msg::IntMessage::SharedPtr message_) -> void{
        // message_->dataを16進数形式で表示 (0埋めはしない)
        std::ostringstream oss;
        for (const auto& byte : message_->data)
        {
            oss << std::hex << (int)byte << " ";
        }

        RCLCPP_INFO(this->get_logger(), "Data: %s", oss.str().c_str());
      };
        
      rclcpp::QoS qos(rclcpp::KeepLast(10));

      // Subscriber作成
      auto subscriber = create_subscription<publisher_node::msg::IntMessage>(topic_name, qos, callback);
      subscribers_.emplace(topic_name, subscriber);
    }
  }

private:
  // トピックごとのPublisher
  std::unordered_map<std::string, rclcpp::Subscription<publisher_node::msg::IntMessage>::SharedPtr> subscribers_;
  publisher_node::msg::IntMessage::SharedPtr message_;
};

int main(int argc, char * argv[])
{
  auto options = parse_options(argc, argv);
  std::cout << options << "\n" << "Start Subscriber!" << std::endl;

  setvbuf(stdout, NULL, _IONBF, BUFSIZ);
  rclcpp::init(argc, argv);

  auto node = std::make_shared<Subscriber>(options);
  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}