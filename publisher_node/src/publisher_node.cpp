/* 目的: あるDockerコンテナに対し、このプロジェクトがクローン&ビルドされると、そのコンテナには指定されたトピック名を持つpublisherノードが存在することになる */

#include <chrono>
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


class Publisher : public rclcpp::Node
{
public:
  explicit Publisher(const node_options::Options & options)
  : Node(options.node_name)
  {
    // タイマー実行されるイベントハンドラー関数
    auto publish_message =
      [this, options]() -> void  // ラムダ式による関数オブジェクトの定義
      {
        // 送信するメッセージの作成
        std::shared_ptr<std::vector<uint8_t>> msg = std::make_shared<std::vector<uint8_t>>(options.payload_size);

        // 配列の内容を0埋めし、文字列に変換
        std::memset(msg.get(), 0, options.payload_size);
        std_msgs::msg::ByteMultiArray byte_msg;
        byte_msg.data = *msg;

        pub_->publish(std::move(byte_msg));
      };

    // chatterトピックの送信設定
    rclcpp::QoS qos(rclcpp::KeepLast(10));
    pub_ = create_publisher<std_msgs::msg::ByteMultiArray>(options.topic_name, qos);
    // publish_messageの100ミリ秒周期でのタイマー実行
    timer_ = create_wall_timer(std::chrono::milliseconds(options.period_ms), publish_message);
  }

private:
  rclcpp::Publisher<std_msgs::msg::ByteMultiArray>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  auto options = parse_options(argc, argv);
  std::cout << options << "\n" << "Start test!" << std::endl;

  // クライアントライブラリの初期化
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);
  rclcpp::init(argc, argv);

  // talkerノードの生成とスピン開始
  auto node = std::make_shared<Publisher>(options);
  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}