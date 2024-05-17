#define BOOST_BIND_NO_PLACEHOLDERS
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include "pcl_conversions/pcl_conversions.h"
#include <string>
#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>


using std::placeholders::_1;
class Voxelize : public rclcpp::Node
{
  public:
    Voxelize()
    : Node("voxelize")
    {
      this->declare_parameter<std::string>("cloud_topic", "/local_grid");
      this->get_parameter("cloud_topic", cloud_topic_);
      
      this->declare_parameter<std::string>("frame_id", "body");
      this->get_parameter("frame_id", frame_id_);

      this->declare_parameter<float>("voxel_size", 0.1);
      this->get_parameter("voxel_size", voxel_size_);

      this->declare_parameter<std::string>("published_topic_name", "local_grid_voxelized");
      this->get_parameter("published_topic_name", published_topic_name_);
      
      sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      cloud_topic_, 10, std::bind(&Voxelize::callback, this, _1));

      
      pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(published_topic_name_, 1);
    }

  private:
    void callback(const sensor_msgs::msg::PointCloud2& cloud_in)
        {
            pcl::PCLPointCloud2::Ptr cloud_voxelized (new pcl::PCLPointCloud2 ());
            sensor_msgs::msg::PointCloud2 cloud_ros;

            // Convert cloud from ROS msg to PCL msg
            pcl_conversions::toPCL(cloud_in, *cloud_voxelized);

            // Create the filtering object
            pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
            sor.setInputCloud (cloud_voxelized);
            sor.setLeafSize (voxel_size_, voxel_size_, voxel_size_);
            sor.filter (*cloud_voxelized);
            
            // Convert back to ROS msg
            pcl_conversions::fromPCL(*cloud_voxelized, cloud_ros);
            cloud_ros.header.frame_id = frame_id_;
            cloud_ros.header.stamp = cloud_in.header.stamp;

            // Publish cloud
            pub_->publish(cloud_ros);
        }
    std::string cloud_topic_, frame_id_, published_topic_name_;
    float voxel_size_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Voxelize>());
  rclcpp::shutdown();
  return 0;
}