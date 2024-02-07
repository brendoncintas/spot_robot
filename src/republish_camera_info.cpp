#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

class RepubishCamInfo : public rclcpp::Node
{
  public:
    RepubishCamInfo()
    : Node("republish_camera_info")
    {
      this->declare_parameter("topic_sub_name", "/camera/camera_info");
      this->declare_parameter("topic_pub_name", "/camera_info");
      std::string topic_sub = this->get_parameter("topic_sub_name").as_string();
      std::string topic_pub = this->get_parameter("topic_pub_name").as_string();
      publisher_ = this->create_publisher<sensor_msgs::msg::CameraInfo>(topic_pub, 10);

      subscription_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
      topic_sub, 10, std::bind(&RepubishCamInfo::topic_callback, this, _1));
    }

  private:
    void topic_callback(const sensor_msgs::msg::CameraInfo & msg) const
    {
      sensor_msgs::msg::CameraInfo CamInfo;
      CamInfo = msg;
      std::vector<double> d = {0, 0, 0, 0, 0};
      CamInfo.d =  d;
      publisher_->publish(CamInfo);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RepubishCamInfo>());
  rclcpp::shutdown();
  return 0;
}