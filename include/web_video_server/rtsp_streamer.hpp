// Copyright (c) 2024, The Robot Web Tools Contributors
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or distribution with the distribution.
//
//    * Neither the name of the copyright holder nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#pragma once

#include <chrono>
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
#include "rtsp_async_server_cpp/rtsp_server.hpp"
#include "rtsp_async_server_cpp/media_stream.hpp"
#include "web_video_server/subscribers/image_transport_subscriber.hpp"
#include "web_video_server/subscribers/pointcloud2_subscriber.hpp"

namespace web_video_server
{

class RTSPStreamer
{
public:
  RTSPStreamer(
    rclcpp::Node::SharedPtr node,
    const std::string & topic,
    const std::string & codec_name = "h264",
    int rtsp_port = 8554);

  ~RTSPStreamer();

  void start();
  void stop();
  
  std::string getTopic() const { return topic_; }
  std::string getStreamUrl() const;
  bool isActive() const { return active_; }
  int getPort() const { return rtsp_port_; }
  
  // Dynamic topic switching
  void switchTopic(const std::string& new_topic);
  bool isStreaming() const { return streaming_; }

private:
  // RTSP request handler for the async server
  bool handleRTSPRequest(
    const rtsp_async_server_cpp::RTSPRequest& request,
    std::shared_ptr<rtsp_async_server_cpp::RTSPConnection> connection);
  
  void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr & msg);
  uint32_t getTimeStamp();
  
  rclcpp::Node::SharedPtr node_;
  std::string topic_;
  std::string codec_name_;
  int rtsp_port_;
  
  std::shared_ptr<RosSubscriber> subscriber_;
  std::map<std::string, std::shared_ptr<SubscriberType>> subscriber_types_;
  
  // RTSP server from rtsp_async_server_cpp
  std::shared_ptr<rtsp_async_server_cpp::RTSPServer> rtsp_server_;
  std::atomic<bool> active_;
  std::atomic<bool> streaming_;
  
  // Stream parameters
  int width_;
  int height_;
  int fps_;
  int bitrate_;
  
  std::chrono::steady_clock::time_point start_time_;
};

class RTSPStreamerManager
{
public:
  RTSPStreamerManager(rclcpp::Node::SharedPtr node);
  ~RTSPStreamerManager();
  
  std::shared_ptr<RTSPStreamer> createStreamer(
    const std::string & topic,
    const std::string & codec = "h264",
    int rtsp_port = 0);  // 0 = auto-assign port
  
  void removeStreamer(const std::string & topic);
  std::shared_ptr<RTSPStreamer> getStreamer(const std::string & topic);
  
  std::vector<std::string> getActiveStreams() const;
  void cleanup();
  
private:
  rclcpp::Node::SharedPtr node_;
  std::map<std::string, std::shared_ptr<RTSPStreamer>> streamers_;
  mutable std::mutex streamers_mutex_;
  int next_port_;
  
  rclcpp::TimerBase::SharedPtr cleanup_timer_;
  void cleanupInactiveStreamers();
};

}  // namespace web_video_server
