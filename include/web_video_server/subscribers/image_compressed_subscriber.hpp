
#ifndef COMPRESSED_IMAGE_SUBSCRIBER_H_
#define COMPRESSED_IMAGE_SUBSCRIBER_H_

#include <sensor_msgs/msg/compressed_image.hpp>

#include <web_video_server/subscribers/ros_subscriber.hpp>

namespace web_video_server
{

class ImageCompressedSubscriber : public RosSubscriber
{    

    typedef std::function<void(const sensor_msgs::msg::CompressedImage::ConstPtr&)> CompressedImageCallback; 

  public:
    ImageCompressedSubscriber(rclcpp::Node::SharedPtr node);

    ~ImageCompressedSubscriber();

    virtual void subscribe(const async_web_server_cpp::HttpRequest &request,
                           const std::string& topic, 
                           const CompressedImageCallback& callback);    
    
    void subscriberCallback(const sensor_msgs::msg::CompressedImage::ConstPtr &input_msg);
  
  private:
    CompressedImageCallback callback_;

    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr ros_sub_;
};

class ImageCompressedSubscriberType : public SubscriberType
{
  public:
    std::shared_ptr<RosSubscriber> create_subscriber(rclcpp::Node::SharedPtr node);
};

} //web_video_server

#endif //COMPRESSED_IMAGE_SUBSCRIBER_H_