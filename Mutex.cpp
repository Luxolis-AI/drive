#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <mutex>
#include <thread>
#include <atomic>

class BlockingServiceNode : public rclcpp::Node
{
public:
  BlockingServiceNode()
  : Node("blocking_service_node"),
    stop_flag_(false)
  {
    service_ = this->create_service<std_srvs::srv::Trigger>(
      "start_blocking_service",
      std::bind(&BlockingServiceNode::handle_service, this, std::placeholders::_1, std::placeholders::_2));

    stop_service_ = this->create_service<std_srvs::srv::Trigger>(
      "stop_service",
      std::bind(&BlockingServiceNode::handle_stop_service, this, std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(this->get_logger(), "Blocking service node has been started.");
  }

private:
  void handle_service(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                      std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    (void)request;  // Unused parameter

    RCLCPP_INFO(this->get_logger(), "Starting blocking operation.");

    while (rclcpp::ok()) {
      {
        std::lock_guard<std::mutex> lock(mutex_);
        if (stop_flag_) {
          RCLCPP_INFO(this->get_logger(), "Stopping blocking operation.");
          stop_flag_ = false; // Reset the flag
          response->success = true;
          response->message = "Operation stopped.";
          return;
        }
      }

      // Simulate long-running operation
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    response->success = false;
    response->message = "Operation failed.";
  }

  void handle_stop_service(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                           std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    (void)request;  // Unused parameter

    std::lock_guard<std::mutex> lock(mutex_);
    stop_flag_ = true;
    response->success = true;
    response->message = "Stop signal received.";
  }

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_service_;
  std::mutex mutex_;
  bool stop_flag_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<BlockingServiceNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
