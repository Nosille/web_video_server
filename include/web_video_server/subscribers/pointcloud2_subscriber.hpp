
#ifndef POINTCLOUD2_SUBSCRIBER_H_
#define POINTCLOUD2_SUBSCRIBER_H_

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <web_video_server/subscribers/ros_subscriber.hpp>

namespace web_video_server
{

class PointCloud2Subscriber : public RosSubscriber
{    
  public:
    PointCloud2Subscriber(rclcpp::Node::SharedPtr node);

    ~PointCloud2Subscriber();

    virtual void subscribe(const async_web_server_cpp::HttpRequest &request,
                           const std::string& topic, 
                           const ImageCallback& callback);    
    
    void subscriberCallback(const sensor_msgs::msg::PointCloud2::ConstPtr &input_msg);
    
    static bool compareFieldsOffset(sensor_msgs::msg::PointField& field1, sensor_msgs::msg::PointField& field2);
    static inline int sizeOfPointField(int datatype);
    
  private:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr ros_sub_;

    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

    geometry_msgs::msg::TransformStamped transform_optical_;

    std::string frame_id_;
    double wait_for_tf_delay_;
    std::string field_;
    int height_, width_, pixel_size_;
    double focal_length_;
};

class PointCloud2SubscriberType : public SubscriberType
{
  public:
    std::shared_ptr<RosSubscriber> create_subscriber(rclcpp::Node::SharedPtr node);
};

} //web_video_server

#endif //POINTCLOUD2_SUBSCRIBER_H_