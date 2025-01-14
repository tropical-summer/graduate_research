#include <chrono>
#include <cstdio>
#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <unordered_map>

#include "node_options_intermediate/cli_options.hpp"
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

class PubSub : public rclcpp::Node
{
  public:
    explicit PubSub(const node_options::Options & options)
    : Node(options.node_name)
    {
      // まずはPub
    for (size_t i = 0; i < options.topic_names_pub.size(); ++i) {
        const std::string & topic_name = options.topic_names_pub[i];
        int payload_size = options.payload_size[i];
        int period_ms = options.period_ms[i];

        pub_idx_[topic_name] = 0;
        start_time_pub_[topic_name] = this->get_clock()->now();

        // Qos設定
        rclcpp::QoS qos(rclcpp::KeepLast(10));

        // 各トピックに対し、単独のpubか、pub/sub兼任のpubかで分ける。

        // 単独なら、通常通りtimerでpub
        if (std::find(options.topic_names_sub.begin(), options.topic_names_sub.end(), topic_name) == options.topic_names_sub.end()) {
            auto publish_message =
            [this, topic_name, payload_size, &options]() -> void
            {
                int current_pub_idx = pub_idx_[topic_name];
                
                // 送信するメッセージの作成
                auto message_ = std::make_shared<publisher_node::msg::IntMessage>();
                message_->data.resize(payload_size);
                std::fill(message_->data.begin(), message_->data.end(), 0);

                // 送信するメッセージにタイムスタンプをつける
                auto time_stamp = this->get_clock()->now();
                if((time_stamp.seconds() - start_time_pub_[topic_name].seconds()) >= options.eval_time) {
                RCLCPP_INFO(this->get_logger(), "Topic %s has reached the evaluation time.", topic_name.c_str());
                timers_[topic_name]->cancel();
                return;
                }

                message_->header.stamp.sec = static_cast<int32_t>(time_stamp.seconds() - start_time_pub_[topic_name].seconds());
                message_->header.stamp.nanosec = static_cast<uint32_t>((time_stamp.nanoseconds() - start_time_pub_[topic_name].nanoseconds()) % 1000000000);
                message_->header.pub_idx = current_pub_idx;
                message_->header.node_name = options.node_name;

                // message->dataを16進数形式で表示 (0埋めはしない)
                std::ostringstream oss;
                for (const auto& byte : message_->data) {
                oss << std::hex << (int)byte << " ";
                }

                RCLCPP_INFO(this->get_logger(), "Topic: %s, Data: %s, Index: %d", topic_name.c_str(), oss.str().c_str(), current_pub_idx);

                // 該当トピックのPublisherでメッセージ送信
                publishers_[topic_name]->publish(*message_);

                pub_idx_[topic_name]++;
            };

            // Publisher作成
            auto publisher = create_publisher<publisher_node::msg::IntMessage>(topic_name, qos);
            publishers_.emplace(topic_name, publisher);

            // Timer作成
            auto timer = create_wall_timer(std::chrono::milliseconds(period_ms), publish_message);
            timers_.emplace(topic_name, timer);
        }

        // 兼任なら、timerでのpubはせずsubからのcallbackを待つ
        else {
            // Publisher作成
            auto publisher = create_publisher<publisher_node::msg::IntMessage>(topic_name, qos);
            publishers_.emplace(topic_name, publisher);
        }

    }

    // Subscriberの宣言
    for (size_t i = 0; i < options.topic_names_sub.size(); ++i) {
        const std::string & topic_name = options.topic_names_sub[i];
        start_time_sub_[topic_name] = this->get_clock()->now();

        rclcpp::QoS qos(rclcpp::KeepLast(10));

        // 各トピックに対し、単独のsubか、pub/sub兼任のsubかで分ける。
        // `callback`を事前に宣言
        std::function<void(const publisher_node::msg::IntMessage::SharedPtr)> callback;

        // 単独なら、受け取ったものを表示するだけのcallback
        if (std::find(options.topic_names_pub.begin(), options.topic_names_pub.end(), topic_name) == options.topic_names_pub.end()) {
            auto callback = [this, topic_name, options](const publisher_node::msg::IntMessage::SharedPtr message_) -> void
            {
                // eval_time秒過ぎてたら受け取らず終了
                auto now = this->get_clock()->now();
                if((now.seconds() - start_time_sub_[topic_name].seconds()) >= options.eval_time) {
                    RCLCPP_INFO(this->get_logger(), "Topic %s has reached the evaluation time.", topic_name.c_str());
                    return;
                }

                // message_->dataを16進数形式で表示 (0埋めはしない)
                std::ostringstream oss;
                for (const auto& byte : message_->data)
                {
                    oss << std::hex << (int)byte << " ";
                }
                // subした時刻などを表示
                oss << std::dec <<"Time: " << (now.seconds() - start_time_sub_[topic_name].seconds()) << "." << (now.nanoseconds() - start_time_sub_[topic_name].nanoseconds());
                int current_pub_idx = message_->header.pub_idx;
                RCLCPP_INFO(this->get_logger(), "Subscribed/ Topic: %s Data: %s Index: %d", topic_name.c_str(), oss.str().c_str(), current_pub_idx);
            };

            // Subscriber作成
            auto subscriber = create_subscription<publisher_node::msg::IntMessage>(topic_name, qos, callback);
            subscribers_.emplace(topic_name, subscriber);
        } 
        // 兼任なら、受け取ったものを表示するだけでなく、同トピックのpubをpublishさせるcallback
        else {
            auto callback = 
            [this, topic_name, options](const publisher_node::msg::IntMessage::SharedPtr message_) -> void
            {
                // 無限ループを避けるため、自身からのpubは受け取らない
                auto publisher_name = message_->header.node_name;
                if(publisher_name == options.node_name) {
                    return;
                }

                auto now = this->get_clock()->now();
                if((now.seconds() - start_time_sub_[topic_name].seconds()) >= options.eval_time) {
                    RCLCPP_INFO(this->get_logger(), "Topic %s has reached the evaluation time.", topic_name.c_str());
                    return;
                }

                // message_->dataを16進数形式で表示 (0埋めはしない)
                std::ostringstream oss;
                for (const auto& byte : message_->data)
                {
                    oss << std::hex << (int)byte << " ";
                }
                // subした時刻などを表示
                oss << std::dec <<"Time: " << (now.seconds() - start_time_sub_[topic_name].seconds()) << "." << (now.nanoseconds() - start_time_sub_[topic_name].nanoseconds());
                int current_pub_idx = message_->header.pub_idx;
                RCLCPP_INFO(this->get_logger(), "Subscribed/ Topic: %s Data: %s Index: %d", topic_name.c_str(), oss.str().c_str(), current_pub_idx);

                // ヘッダのタイムスタンプを書き換え
                message_->header.stamp.sec = static_cast<int32_t>(now.seconds() - start_time_sub_[topic_name].seconds());
                message_->header.stamp.nanosec = static_cast<uint32_t>((now.nanoseconds() - start_time_sub_[topic_name].nanoseconds()) % 1000000000);
                message_->header.node_name = options.node_name;

                // 同じトピックを扱うpublisherを起動
                oss.str("");
                oss.clear();
                for (const auto& byte : message_->data)
                {
                    oss << std::hex << (int)byte << " ";
                }
                RCLCPP_INFO(this->get_logger(), "Publish/ Topic: %s Data: %s Index: %d", topic_name.c_str(), oss.str().c_str(), current_pub_idx);
                publishers_[topic_name]->publish(*message_);
            };

            // Subscriber作成
            auto subscriber = create_subscription<publisher_node::msg::IntMessage>(topic_name, qos, callback);
            subscribers_.emplace(topic_name, subscriber);
        }
    }

    }


  private:
    std::unordered_map<std::string, rclcpp::Publisher<publisher_node::msg::IntMessage>::SharedPtr> publishers_;
    std::unordered_map<std::string, rclcpp::Subscription<publisher_node::msg::IntMessage>::SharedPtr> subscribers_;

    std::unordered_map<std::string, uint32_t> pub_idx_;
    std::unordered_map<std::string, rclcpp::TimerBase::SharedPtr> timers_;
    std::unordered_map<std::string, rclcpp::Time> start_time_pub_;
    std::unordered_map<std::string, rclcpp::Time> start_time_sub_;
};

int main(int argc, char * argv[])
{
  auto options = parse_options(argc, argv);
  std::cout << options << "\n" << "Start Publisher & Subscriber!" << std::endl;

  // クライアントライブラリの初期化
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);
  rclcpp::init(argc, argv);

  // Publisherノードの生成とスピン開始
  auto node = std::make_shared<PubSub>(options);
  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}