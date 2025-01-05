#include <chrono>
#include <iostream>
#include <string>
#include <vector>

#include "cxxopts.hpp"
#include "node_options/cli_options.hpp"

namespace node_options
{

// デフォルト値
Options::Options()
{
    payload_size = 32;
    period_ms = 10;
}

// コンストラクタ
Options::Options(int argc, char ** argv)
: Options()
{
    parse(argc, argv);
}

// 受け取ったコマンドライン引数をもとに、option変数を更新
void Options::parse(int argc, char ** argv)
{
    // コマンドラインのargsをパース
    cxxopts::Options options(argv[0], "ROS2 performance benchmark");

    options.add_options()(
        "node_name", "name for this node", cxxopts::value<std::string>(node_name))(
        "topic_name", "topic_name for this node", cxxopts::value<std::string>(topic_name))(
        "s, size", "payload size", cxxopts::value<int>(payload_size)->default_value(std::to_string(payload_size)), "bytes")(
        "p, period", "publish frequency", cxxopts::value<int>(period_ms)->default_value(std::to_string(period_ms)), "ms_sec"    
        );

    try {
        auto result = options.parse(argc, argv);

        if(result.count("node_name") == 0){
            std::cout << "Please specify the name for this node" << std::endl;
            exit(1);
        }

        if(result.count("topic_name") == 0){
            std::cout << "Please specify the topic_name for this node" << std::endl;
            exit(1);
        }
    } catch (const cxxopts::exceptions::exception & e) {
        std::cout << "Error parsing options: " << e.what() << std::endl;
        exit(1);
    }
}

// コマンドラインでの表示を見やすくするためのオーバーロード処理
std::ostream & operator<<(std::ostream & os, const Options & options)
{
    os << "topic_name: " << options.topic_name << std::endl;
    os << "payload_size: " << options.payload_size << " bytes" << std::endl;
    os << "period_ms: " << options.period_ms << " miliseconds" <<  std::endl;

    return os;
}

} // ここまでnode_optionsの名前空間