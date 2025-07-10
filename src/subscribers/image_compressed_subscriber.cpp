
#include "web_video_server/subscribers/image_compressed_subscriber.hpp"

namespace web_video_server
{

ImageCompressedSubscriber::ImageCompressedSubscriber(rclcpp::Node::SharedPtr node)
: RosSubscriber(node)
{
}

ImageCompressedSubscriber::~ImageCompressedSubscriber()
{
}

void ImageCompressedSubscriber::subscribe(const async_web_server_cpp::HttpRequest &request,
                                         const std::string& topic, 
                                         const CompressedImageCallback& callback)
{
  callback_ = callback;
  const std::string compressed_topic = topic + "/compressed";
  qos_profile_name_ = request.get_query_param_value_or_default("qos_profile", "default");

  // Get QoS profile from query parameter
  RCLCPP_INFO(
    node_->get_logger(), "Streaming topic %s with QoS profile %s", topic.c_str(),
    qos_profile_name_.c_str());
  auto qos_profile = get_qos_profile_from_name(qos_profile_name_);
  if (!qos_profile) {
    qos_profile = rmw_qos_profile_default;
    RCLCPP_ERROR(
      node_->get_logger(),
      "Invalid QoS profile %s specified. Using default profile.",
      qos_profile_name_.c_str());
  }

  const auto qos = rclcpp::QoS(
    rclcpp::QoSInitialization(qos_profile.value().history, 1),
    qos_profile.value());

  ros_sub_ = node_->create_subscription<sensor_msgs::msg::CompressedImage>(compressed_topic, qos, std::bind(&ImageCompressedSubscriber::subscriberCallback, this, std::placeholders::_1));
}

void ImageCompressedSubscriber::subscriberCallback(const sensor_msgs::msg::CompressedImage::ConstPtr &input_msg)
{
  callback_(input_msg);
}


std::shared_ptr<RosSubscriber> ImageCompressedSubscriberType::create_subscriber(rclcpp::Node::SharedPtr node)
{
  return std::shared_ptr<RosSubscriber>(
      new ImageCompressedSubscriber(node));
}


}