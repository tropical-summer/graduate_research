/* 目的: あるDockerコンテナに対し、このプロジェクトがクローン&ビルドされると、そのコンテナには指定されたトピック名を持つpublisherノードが存在することになる */

#include <chrono>
#include <cstdio>
#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/byte_multi_array.hpp>

#include "node_options/cli_options.hpp"
#include "publisher_node/msg/performance_header.hpp"
#include "publisher_node/msg/int_message.hpp"

// コマンドラインオプション
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

class Publisher : public rclcpp::Node
{
public:
  explicit Publisher(const node_options::Options & options)
  : Node(options.node_name)
  {
    // タイマー実行されるイベントハンドラー関数
    auto publish_message =
      [this, options]() -> void  
      {
        // 送信するメッセージの作成
        auto message_ = std::make_shared<publisher_node::msg::IntMessage>();
        message_->data.resize(options.payload_size);
        std::fill(message_->data.begin(), message_->data.end(), 0);

        // message_->dataを16進数形式で表示 (0埋めはしない)
        std::ostringstream oss;
        for (const auto& byte : message_->data)
        {
            oss << std::hex << (int)byte << " ";
        }

        // RCLCPP_INFOで16進数データを表示
        RCLCPP_INFO(this->get_logger(), "Data: %s", oss.str().c_str());

        pub_->publish(*message_);
      };

    // Qos設定
    rclcpp::QoS qos(rclcpp::KeepLast(10));

    // publish_messageのPERIOD_MS周期でのタイマー実行
    pub_ = create_publisher<publisher_node::msg::IntMessage>(options.topic_name, qos);
    timer_ = create_wall_timer(std::chrono::milliseconds(options.period_ms), publish_message);
  }

private:
  rclcpp::Publisher<publisher_node::msg::IntMessage>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  publisher_node::msg::IntMessage::SharedPtr message_;
};

int main(int argc, char * argv[])
{
  auto options = parse_options(argc, argv);
  std::cout << options << "\n" << "Start Publisher!" << std::endl;

  // クライアントライブラリの初期化
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);
  rclcpp::init(argc, argv);

  // talkerノードの生成とスピン開始
  auto node = std::make_shared<Publisher>(options);
  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}