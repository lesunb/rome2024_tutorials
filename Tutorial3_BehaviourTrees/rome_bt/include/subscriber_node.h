#include "rclcpp/rclcpp.hpp"
#include <rclcpp/qos.hpp>
#include "std_msgs/msg/string.hpp"

#include "behaviortree_cpp_v3/action_node.h"

#include <string>


class SubscriberNode: public BT::ConditionNode{
    public:
        explicit SubscriberNode(const std::string &name,
                               const BT::NodeConfiguration &config,
                               rclcpp::Node::SharedPtr node_ptr);

        BT::NodeStatus tick() override;

        static BT::PortsList providedPorts() {
            return BT::PortsList {BT::OutputPort<std::string>("message"),
                                  BT::InputPort<int>("wait_duration_sec")};
        }

        void update_msg(const std_msgs::msg::String::SharedPtr new_msg);
        bool wait_for_message(int wait_duration);


    private:
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr str_subscriber_ptr_;
        std::string msg_;
};
