/* 目的: あるDockerコンテナに対し、このプロジェクトがクローン&ビルドされると、そのコンテナには指定されたトピック名を持つpublisherノードが存在することになる */
/*  */

#include "rclcpp/rclcpp.hpp"

class Publisher : public rclcpp::Node, public QoS
{
};
