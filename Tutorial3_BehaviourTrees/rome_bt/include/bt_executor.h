#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/utils/shared_library.h"
#include "ament_index_cpp/get_package_share_directory.hpp"

#include <string>
#include <fstream>
#include <chrono>


class BTExecutor : public rclcpp::Node
{
public:
  explicit BTExecutor(const std::string &node_name);
  void setup();
  void create_behavior_tree();
  void update_behavior_tree();
  void halt_behavior_tree();

  void register_nav2_plugins();

private:
  BT::BehaviorTreeFactory factory_;
  rclcpp::TimerBase::SharedPtr timer_;
  BT::Tree tree_;
};
