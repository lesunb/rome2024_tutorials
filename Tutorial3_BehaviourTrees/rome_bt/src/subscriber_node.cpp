#include "subscriber_node.h"


SubscriberNode::SubscriberNode(const std::string &xml_tag_name,
                             const BT::NodeConfiguration &conf,
                             rclcpp::Node::SharedPtr node_ptr): BT::ConditionNode(xml_tag_name, conf){
    std::string topic = "hello";

    auto qos = rclcpp::SystemDefaultsQoS();
    qos.best_effort();
    str_subscriber_ptr_ = node_ptr->create_subscription<std_msgs::msg::String>(topic, qos,
                                                                               std::bind(&SubscriberNode::update_msg, this, std::placeholders::_1));
}

void SubscriberNode::update_msg(const std_msgs::msg::String::SharedPtr new_msg){
    msg_ = new_msg->data;
}

// Waits for given duration or until message is received
bool SubscriberNode::wait_for_message(int wait_duration){
    rclcpp::WaitSet wait_set;
    wait_set.add_subscription(str_subscriber_ptr_);
    auto ret = wait_set.wait(std::chrono::seconds(wait_duration));
    if (ret.kind() == rclcpp::WaitResultKind::Ready) {
        std_msgs::msg::String msg;
        rclcpp::MessageInfo info;
        auto ret_take = str_subscriber_ptr_->take(msg, info);
        return (bool) ret_take;
    }
    return false;
}

BT::NodeStatus SubscriberNode::tick(){
    auto wait_duration_sec = getInput<int>("wait_duration_sec");
    int wait_duration = wait_duration_sec.value();
    bool received_msg = wait_for_message(wait_duration);

    if (msg_.empty() || !received_msg)
        return BT::NodeStatus::FAILURE;

    setOutput("message", msg_);
    return BT::NodeStatus::SUCCESS;
}
