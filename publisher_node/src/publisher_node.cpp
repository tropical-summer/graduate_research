/* 目的: あるDockerコンテナに対し、このプロジェクトがクローン&ビルドされると、そのコンテナには指定されたトピック名を持つpublisherノードが存在することになる */

#include <chrono>
#include <cstdio>
#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <unordered_map>
#include <sstream>
#include <fstream>
#include <filesystem>

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

static
void
create_result_directory(const node_options::Options & options)
{
  std::stringstream ss;
  ss << options.node_name << "_log" ;
  const std::string result_dir_name = ss.str();
  std::filesystem::create_directories(result_dir_name); // install/publisher_node/lib/publisher_node/my_node_log みたいな感じ
  ss.str("");
  ss.clear();

  std::vector<std::string> log_file_paths;
  for (size_t i = 0; i < options.topic_names.size(); ++i) {
    ss << result_dir_name << "/" << options.topic_names[i] << "_log.txt";
    std::string log_file_path = ss.str();
    log_file_paths.push_back(log_file_path);
    ss.str("");
    ss.clear();
  }

  for (const auto& file_path : log_file_paths) {
    std::ofstream ofs(file_path); // ファイルを開く（存在しない場合は作成）
    if(ofs){
      std::cout << "Log file created: " << file_path << std::endl;
      ofs.close();
    } else {
      std::cerr << "Failed to create: " << file_path << std::endl;
    }
  }
}

class Publisher : public rclcpp::Node
{
  public:
    explicit Publisher(const node_options::Options & options)
    : Node(options.node_name)
    {
      // 複数のトピック名を扱う場合
      for (size_t i = 0; i < options.topic_names.size(); ++i) {
        const std::string & topic_name = options.topic_names[i];
        int payload_size = options.payload_size[i];
        int period_ms = options.period_ms[i];

        pub_idx_[topic_name] = 0;
        start_time_[topic_name] = this->get_clock()->now();

        // タイマー実行されるイベントハンドラー関数を生成
        auto publish_message =
          [this, topic_name, payload_size, &options]() -> void
          {
            int current_pub_idx = pub_idx_[topic_name];
            
            // 送信するメッセージの作成
            auto message_ = std::make_shared<publisher_node::msg::IntMessage>();
            message_->data.resize(payload_size);
            std::fill(message_->data.begin(), message_->data.end(), 0);

            auto time_stamp = this->get_clock()->now();
            if((time_stamp.seconds() - start_time_[topic_name].seconds()) >= options.eval_time) {
              RCLCPP_INFO(this->get_logger(), "Topic %s has reached the evaluation time.", topic_name.c_str());
              timers_[topic_name]->cancel();
              end_time_[topic_name] = this->get_clock()->now();
              return;
            }

            message_->header.stamp.sec = static_cast<int32_t>(time_stamp.seconds() - start_time_[topic_name].seconds());
            message_->header.stamp.nanosec = static_cast<uint32_t>((time_stamp.nanoseconds() - start_time_[topic_name].nanoseconds()) % 1000000000);
            message_->header.pub_idx = current_pub_idx;
            message_->header.node_name = options.node_name;

            // message->dataを16進数形式で表示 (0埋めはしない)
            std::ostringstream oss;
            for (const auto& byte : message_->data) {
              oss << std::hex << (int)byte << " ";
            }
            oss << std::dec <<"Time: " << std::fixed << std::setprecision(9) << static_cast<double>(time_stamp.nanoseconds() - start_time_[topic_name].nanoseconds()) / 1e9;

            RCLCPP_INFO(this->get_logger(), "Publish/ Topic: %s, Data: %s, Index: %d", topic_name.c_str(), oss.str().c_str(), current_pub_idx);

            // 該当トピックのPublisherでメッセージ送信
            publishers_[topic_name]->publish(*message_);

            pub_idx_[topic_name]++;
        };

        // Qos設定
        rclcpp::QoS qos(rclcpp::KeepLast(10));

        // Publisher作成
        auto publisher = create_publisher<publisher_node::msg::IntMessage>(topic_name, qos);
        publishers_.emplace(topic_name, publisher);

        // Timer作成
        auto timer = create_wall_timer(std::chrono::milliseconds(period_ms), publish_message);
        timers_.emplace(topic_name, timer);
      }
    }


  private:
    // トピックごとのPublisher,Timer,
    std::unordered_map<std::string, rclcpp::Publisher<publisher_node::msg::IntMessage>::SharedPtr> publishers_;

    // タイマーを保持
    std::unordered_map<std::string, rclcpp::TimerBase::SharedPtr> timers_;

    std::unordered_map<std::string, uint32_t> pub_idx_;
    std::unordered_map<std::string, rclcpp::Time> start_time_;
    std::unordered_map<std::string, rclcpp::Time> end_time_;
};

int main(int argc, char * argv[])
{
  auto options = parse_options(argc, argv);
  create_result_directory(options) ;
  std::cout << options << "\n" << "Start Publisher!" << std::endl;

  // クライアントライブラリの初期化
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);
  rclcpp::init(argc, argv);

  // Publisherノードの生成とスピン開始
  auto node = std::make_shared<Publisher>(options);
  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}