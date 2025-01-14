#include <cstdio>
#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <sstream>
#include <fstream>
#include <filesystem>

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

static
void
create_result_directory(const node_options::Options & options)
{
  std::stringstream ss;
  ss << options.node_name << "_log" ;
  const std::string result_dir_name = ss.str();
  std::filesystem::create_directories(result_dir_name); 
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

class Subscriber : public rclcpp::Node
{
public:
  explicit Subscriber(const node_options::Options & options)
    : Node(options.node_name)
  {
    // 複数のトピック名を扱う場合
    for (size_t i = 0; i < options.topic_names.size(); ++i) {
      const std::string & topic_name = options.topic_names[i];
      start_time_[topic_name] = this->get_clock()->now();

      auto callback = [this, topic_name, options](const publisher_node::msg::IntMessage::SharedPtr message_) -> void{
        // eval_time秒過ぎてたら受け取らず終了
        auto sub_time = this->get_clock()->now();
        if((sub_time.seconds() - start_time_[topic_name].seconds()) >= options.eval_time) {
          RCLCPP_INFO(this->get_logger(), "Topic %s has reached the evaluation time.", topic_name.c_str());
          end_time_[topic_name] = this->get_clock()->now();
          return;
        }

        // message_->dataを16進数形式で表示 (0埋めはしない)
        std::ostringstream oss;
        for (const auto& byte : message_->data)
        {
            oss << std::hex << (int)byte << " ";
        }
        // subした時刻などを表示
        oss << std::dec <<"Time: " << std::fixed << std::setprecision(9) << static_cast<double>(sub_time.nanoseconds() - start_time_[topic_name].nanoseconds()) / 1e9;
        int current_pub_idx = message_->header.pub_idx;
        RCLCPP_INFO(this->get_logger(), "Subscribe/ Topic: %s Data: %s Index: %d", topic_name.c_str(), oss.str().c_str(), current_pub_idx);
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
  std::unordered_map<std::string, rclcpp::Time> start_time_;
  std::unordered_map<std::string, rclcpp::Time> end_time_;
};

int main(int argc, char * argv[])
{
  auto options = parse_options(argc, argv);
  create_result_directory(options) ;
  std::cout << options << "\n" << "Start Subscriber!" << std::endl;

  setvbuf(stdout, NULL, _IONBF, BUFSIZ);
  rclcpp::init(argc, argv);

  auto node = std::make_shared<Subscriber>(options);
  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}