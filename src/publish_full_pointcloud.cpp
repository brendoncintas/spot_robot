#define BOOST_BIND_NO_PLACEHOLDERS
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include "pcl_conversions/pcl_conversions.h"
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include "tf2_ros/transform_listener.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/buffer.h"
#include "tf2_sensor_msgs/tf2_sensor_msgs.hpp"

using std::placeholders::_1;
class FusePointclouds : public rclcpp::Node
{
  public:
    FusePointclouds()
    : Node("publish_full_pointcloud")
    {
      tf_buffer_ =
      std::make_unique<tf2_ros::Buffer>(this->get_clock());
      tf_listener_ =
      std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

      this->declare_parameter<std::string>("cloud_topic0", "/depth_registered/hand/points");
      this->get_parameter("cloud_topic0", cloud_topic0_);

      this->declare_parameter<std::string>("cloud_topic1", "/depth_registered/frontleft/points");
      this->get_parameter("cloud_topic1", cloud_topic1_);

      this->declare_parameter<std::string>("cloud_topic2", "/depth_registered/frontright/points");
      this->get_parameter("cloud_topic2", cloud_topic2_);

      this->declare_parameter<std::string>("cloud_topic3", "/depth_registered/left/points");
      this->get_parameter("cloud_topic3", cloud_topic3_);

      this->declare_parameter<std::string>("cloud_topic4", "/depth_registered/right/points");
      this->get_parameter("cloud_topic4", cloud_topic4_);

      this->declare_parameter<std::string>("cloud_topic5", "/depth_registered/back/points");
      this->get_parameter("cloud_topic5", cloud_topic5_);
      
      this->declare_parameter<std::string>("frame_id", "body");
      this->get_parameter("frame_id", frame_id_);
      
      cloud0_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      cloud_topic0_, 10, std::bind(&FusePointclouds::voxelize, this, _1));
      cloud1_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      cloud_topic1_, 10, std::bind(&FusePointclouds::voxelize, this, _1));
      cloud2_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      cloud_topic2_, 10, std::bind(&FusePointclouds::voxelize, this, _1));
      cloud3_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      cloud_topic3_, 10, std::bind(&FusePointclouds::voxelize, this, _1));
      cloud4_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      cloud_topic4_, 10, std::bind(&FusePointclouds::voxelize, this, _1));

      cloud5_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      cloud_topic5_, 10, std::bind(&FusePointclouds::voxelize, this, _1));
      
      publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("full_cloud", 10);
    }

  private:
    void voxelize(const sensor_msgs::msg::PointCloud2& cloud_in) const
        {
            pcl::PCLPointCloud2::Ptr cloud_raw (new pcl::PCLPointCloud2 ());
            pcl::PCLPointCloud2::Ptr cloud_voxelized (new pcl::PCLPointCloud2 ());
            sensor_msgs::msg::PointCloud2 cloud_ros;
            sensor_msgs::msg::PointCloud2 cloud_transformed;

            //Transform to body frame
            geometry_msgs::msg::TransformStamped transform;
            try {
                transform = tf_buffer_->lookupTransform(
                frame_id_, cloud_in.header.frame_id, cloud_in.header.stamp);
            } catch (const tf2::TransformException & ex) {
                RCLCPP_INFO(
                this->get_logger(), "Could not transform %s to %s: %s",
                frame_id_, cloud_in.header.frame_id, ex.what());
            return;
            }
            tf2::doTransform(cloud_in, cloud_transformed, transform);

            // Convert cloud from ROS msg to PCL msg
            pcl_conversions::toPCL(cloud_transformed, *cloud_raw);

            // Create the filtering object
            pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
            sor.setInputCloud (cloud_raw);
            sor.setLeafSize (0.05f, 0.05f, 0.05f);
            sor.filter (*cloud_voxelized);

            //Concatenate
            pcl::concatenateFields(*cloud_voxelized, *cloud_full, *cloud_full);
            
            // Convert back to ROS msg
            pcl_conversions::fromPCL(*cloud_full, cloud_ros);
            cloud_ros.header.frame_id = frame_id_;
            
            // Publish cloud
            publisher_->publish(cloud_ros);
        }
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    pcl::PCLPointCloud2::Ptr cloud_full;
    std::string cloud_topic0_, cloud_topic1_, cloud_topic2_, cloud_topic3_, cloud_topic4_, cloud_topic5_, frame_id_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud0_, cloud1_, cloud2_, cloud3_, cloud4_, cloud5_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FusePointclouds>());
  rclcpp::shutdown();
  return 0;
}