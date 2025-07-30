#pragma once

#include <memory>
#include <string>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <queue>
#include <map>
#include <atomic>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "web_video_server/subscribers/image_transport_subscriber.hpp"
#include "web_video_server/subscribers/pointcloud2_subscriber.hpp"

#include <gst/gst.h>
#include <gst/rtsp-server/rtsp-server.h>

namespace web_video_server
{

class GstRTSPStreamer
{
public:
  GstRTSPStreamer(
    rclcpp::Node::SharedPtr node,
    const std::string & topic,
    const std::string & codec_name = "h264",
    int rtsp_port = 8554);

  ~GstRTSPStreamer();

  void start();
  void stop();

  std::string getTopic() const { return topic_; }
  std::string getStreamUrl() const;
  bool isActive() const { return active_; }
  int getPort() const { return rtsp_port_; }

private:
  void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr & msg);

  rclcpp::Node::SharedPtr node_;
  std::string topic_;
  std::string codec_name_;
  int rtsp_port_;

  std::shared_ptr<RosSubscriber> subscriber_;
  std::map<std::string, std::shared_ptr<SubscriberType>> subscriber_types_;

  GstElement *pipeline_ = nullptr;
  GstElement *appsrc_ = nullptr;
  GMainLoop *main_loop_ = nullptr;
  std::thread gst_thread_;
  GstRTSPServer *rtsp_server_ = nullptr;
  guint server_id_ = 0;

  std::atomic<bool> active_;
};

class GstRTSPStreamerManager
{
public:
  GstRTSPStreamerManager(rclcpp::Node::SharedPtr node);
  ~GstRTSPStreamerManager();

  std::shared_ptr<GstRTSPStreamer> createStreamer(
    const std::string & topic,
    const std::string & codec = "h264",
    int rtsp_port = 0);  // 0 = auto-assign port

  void removeStreamer(const std::string & topic);
  std::shared_ptr<GstRTSPStreamer> getStreamer(const std::string & topic);

  std::vector<std::string> getActiveStreams() const;
  void cleanup();

private:
  rclcpp::Node::SharedPtr node_;
  std::map<std::string, std::shared_ptr<GstRTSPStreamer>> streamers_;
  mutable std::mutex streamers_mutex_;
  int next_port_;

  rclcpp::TimerBase::SharedPtr cleanup_timer_;
  void cleanupInactiveStreamers();
};

}  // namespace web_video_server

