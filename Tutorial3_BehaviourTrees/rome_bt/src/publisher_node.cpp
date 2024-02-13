#include "publisher_node.h"


PublisherNode::PublisherNode(const std::string &xml_tag_name,
                             const BT::NodeConfiguration &conf,
                             rclcpp::Node::SharedPtr node_ptr): BT::SyncActionNode(xml_tag_name, conf){
    std::string topic = "hello";
    auto qos = rclcpp::SystemDefaultsQoS();
    qos.best_effort();
    str_publisher_ptr_ = node_ptr->create_publisher<std_msgs::msg::String>(topic, qos);
}

BT::NodeStatus PublisherNode::tick(){
    std_msgs::msg::String str_msg;
    auto msg = getInput<std::string>("message");
    auto num_input = getInput<int>("number");
    int times_to_send = num_input.value();
    str_msg.data = msg.value();
    for(int i=0; i<times_to_send; i++){
        str_publisher_ptr_->publish(str_msg);
        sleep(1);
    }
    return BT::NodeStatus::SUCCESS;
}

