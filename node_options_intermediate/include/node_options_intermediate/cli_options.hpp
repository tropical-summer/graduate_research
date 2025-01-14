// 二重インクルード防止
#ifndef NODE_OPTIONS__CLI_OPTIONS_HPP_
#define NODE_OPTIONS__CLI_OPTIONS_HPP_

#include <string>
#include <vector>

namespace node_options
{

class Options
{
public:
 Options();

 Options(int argc, char ** argv);

 void parse(int argc, char ** argv);

 std::string node_name;
 std::vector<std::string> topic_names_pub;
 std::vector<std::string> topic_names_sub;
 std::vector<int> payload_size;
 std::vector<int> period_ms;
 int eval_time;
};

std::ostream & operator<<(std::ostream & os, const Options & options);

} // ここまでnode_optionsの名前空間

#endif