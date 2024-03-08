#define BOOST_BIND_NO_PLACEHOLDERS
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include "pcl_conversions/pcl_conversions.h"
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cmath>
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
      cloud_count = 0;
      pub_ref_time = 0;
      cloud_full = pcl::PCLPointCloud2::Ptr(new pcl::PCLPointCloud2 ());
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

      this->declare_parameter<double>("clip_distance", 3);
      this->get_parameter("clip_distance", clip_distance);

      this->declare_parameter<float>("voxel_size", 0.04);
      this->get_parameter("voxel_size", voxel_size);

      this->declare_parameter<int>("msg_batch_size", 10);
      this->get_parameter("msg_batch_size", msg_batch_size);

      this->declare_parameter<std::string>("published_topic_name", "cloud_full");
      this->get_parameter("published_topic_name", published_topic_name);
      
      cloud0_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      cloud_topic0_, rclcpp::SensorDataQoS(), std::bind(&FusePointclouds::combine, this, _1));
      cloud1_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      cloud_topic1_, rclcpp::SensorDataQoS(), std::bind(&FusePointclouds::combine, this, _1));
      cloud2_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      cloud_topic2_, rclcpp::SensorDataQoS(), std::bind(&FusePointclouds::combine, this, _1));
      cloud3_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      cloud_topic3_, rclcpp::SensorDataQoS(), std::bind(&FusePointclouds::combine, this, _1));
      cloud4_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      cloud_topic4_, rclcpp::SensorDataQoS(), std::bind(&FusePointclouds::combine, this, _1));
      cloud5_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      cloud_topic5_, rclcpp::SensorDataQoS(), std::bind(&FusePointclouds::combine, this, _1));
      
      publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(published_topic_name, rclcpp::SensorDataQoS());
    }

  private:
    void combine(const sensor_msgs::msg::PointCloud2& cloud_in)
        {
            //Check if current message is within same timeframe
            if (pub_ref_time == 0){
              pub_ref_time = cloud_in.header.stamp.nanosec;
            }
            if(cloud_in.header.stamp.nanosec - pub_ref_time > 2e+9){
              return;
            }

            pcl::PCLPointCloud2::Ptr cloud_raw (new pcl::PCLPointCloud2 ());
            pcl::PCLPointCloud2::Ptr cloud_clipped (new pcl::PCLPointCloud2 ());
            pcl::PCLPointCloud2::Ptr cloud_voxelized (new pcl::PCLPointCloud2 ());
            sensor_msgs::msg::PointCloud2 cloud_ros;
            sensor_msgs::msg::PointCloud2 cloud_transformed;

            //Convert to PCL, clip distance, convert back to ROS for transformation
            pcl_conversions::toPCL(cloud_in, *cloud_raw);

            pcl::PassThrough<pcl::PCLPointCloud2> pass;
            pass.setInputCloud (cloud_raw);
            pass.setFilterFieldName ("z");
            pass.setFilterLimits (0.0, clip_distance);
            pass.filter (*cloud_clipped);
            pcl_conversions::fromPCL(*cloud_clipped, cloud_ros);

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
            tf2::doTransform(cloud_ros, cloud_transformed, transform);

            // Convert cloud from ROS msg to PCL msg
            pcl_conversions::toPCL(cloud_transformed, *cloud_clipped);

            // If it's the first concatenation attempt, set cloud_full to cloud_voxelized
            if (cloud_full->width == 0 && cloud_full->height == 0) {
                *cloud_full = *cloud_clipped;
            } else {
                *cloud_full += *cloud_clipped;
            }

            // Create the filtering object
            pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
            sor.setInputCloud (cloud_full);
            sor.setLeafSize (voxel_size, voxel_size, voxel_size);
            sor.filter (*cloud_voxelized);
            
            // Convert back to ROS msg
            pcl_conversions::fromPCL(*cloud_voxelized, cloud_ros);
            cloud_ros.header.frame_id = frame_id_;

            cloud_count += 1;
            // Publish cloud
            rclcpp::Time now = this->get_clock()->now();
            if (cloud_count >= msg_batch_size){
              publisher_->publish(cloud_ros);
              pub_ref_time = 0;
              cloud_full = pcl::PCLPointCloud2::Ptr(new pcl::PCLPointCloud2 ());
              cloud_count = 0;
            }
        }
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    pcl::PCLPointCloud2::Ptr cloud_full;
    std::string cloud_topic0_, cloud_topic1_, cloud_topic2_, cloud_topic3_, cloud_topic4_, cloud_topic5_, frame_id_, published_topic_name;
    double clip_distance;
    float voxel_size;
    int pub_ref_time, cloud_count, msg_batch_size;
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