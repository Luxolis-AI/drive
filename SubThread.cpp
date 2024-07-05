#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <thread>
#include <atomic>

class SubscriberInThreadNode : public rclcpp::Node
{
public:
  SubscriberInThreadNode()
  : Node("subscriber_in_thread_node"),
    stop_flag_(false)
  {
    // Start the subscriber thread
    subscriber_thread_ = std::thread(&SubscriberInThreadNode::subscriber_thread_function, this);
  }

  ~SubscriberInThreadNode()
  {
    stop_flag_.store(true);
    if (subscriber_thread_.joinable()) {
      subscriber_thread_.join();
    }
  }

private:
  void subscriber_callback(const std_msgs::msg::String::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "Received message: '%s'", msg->data.c_str());
  }

  void subscriber_thread_function()
  {
    auto sub_node = std::make_shared<rclcpp::Node>("sub_node");
    auto subscription = sub_node->create_subscription<std_msgs::msg::String>(
      "topic",
      10,
      std::bind(&SubscriberInThreadNode::subscriber_callback, this, std::placeholders::_1)
    );

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(sub_node);

    while (!stop_flag_.load()) {
      executor.spin_some();
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    executor.remove_node(sub_node);
  }

  std::thread subscriber_thread_;
  std::atomic<bool> stop_flag_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SubscriberInThreadNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
