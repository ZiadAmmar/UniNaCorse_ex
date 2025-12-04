#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <sysmonitor_interfaces/msg/sysmon.hpp>

class TestPublisher : public rclcpp::Node
{
public:
  TestPublisher() : Node("test_publisher")
  {
    publisher_ = this->create_publisher<std_msgs::msg::Float64>("/test", 10);
    
    subscription_ = this->create_subscription<sysmonitor_interfaces::msg::Sysmon>(
      "/system_info", 10,
      std::bind(&TestPublisher::system_info_callback, this, std::placeholders::_1));
    
    RCLCPP_INFO(this->get_logger(), "Test Publisher node has been started");
  }

private:
  void system_info_callback(const sysmonitor_interfaces::msg::Sysmon::SharedPtr msg)
  {
    (void)msg;  // Suppress unused variable warning
    auto float_msg = std_msgs::msg::Float64();
    float_msg.data = 1.0;
    publisher_->publish(float_msg);
    
    RCLCPP_INFO(this->get_logger(), "Published 1.0 to /test topic");
  }
  
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_;
  rclcpp::Subscription<sysmonitor_interfaces::msg::Sysmon>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TestPublisher>());
  rclcpp::shutdown();
  return 0;
}