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
//      documentation and/or materials provided with the distribution.
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

#include "web_video_server/rtsp_streamer.hpp"

#include "rtsp_async_server_cpp/rtsp_server.hpp"
#include "rtsp_async_server_cpp/media_stream.hpp"
#include "async_web_server_cpp/http_request.hpp"
#include <boost/bind.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <sensor_msgs/image_encodings.hpp>

using namespace std::chrono_literals;

namespace web_video_server
{

RTSPStreamer::RTSPStreamer(
  rclcpp::Node::SharedPtr node,
  const std::string& topic,
  const std::string& codec_name,
  int rtsp_port)
: node_(node),
  topic_(topic),
  codec_name_(codec_name),
  rtsp_port_(rtsp_port),
  active_(false),
  streaming_(false),
  width_(640),
  height_(480),
  fps_(30),
  bitrate_(1000000)
{
  // Initialize subscriber types
  subscriber_types_["image"] = std::make_shared<ImageTransportSubscriberType>();
  subscriber_types_["pointcloud2"] = std::make_shared<PointCloud2SubscriberType>();

  start_time_ = std::chrono::steady_clock::now();

  // Create RTSP server and media stream
  try {
    rtsp_server_ = std::make_shared<rtsp_async_server_cpp::RTSPServer>(
      "0.0.0.0", std::to_string(rtsp_port),
      boost::bind(&RTSPStreamer::handleRTSPRequest, this, _1, _2),
      2);

    // Create H264 media stream
    auto media_stream = std::make_shared<rtsp_async_server_cpp::H264MediaStream>(
      topic_, width_, height_, fps_, bitrate_);
    rtsp_server_->addMediaStream(topic_, media_stream);
  } catch (const std::exception& e) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to create RTSP server: %s", e.what());
  }
}

RTSPStreamer::~RTSPStreamer()
{
  stop();
}

void RTSPStreamer::start()
{
  if (active_) {
    return;
  }

  RCLCPP_INFO(node_->get_logger(), "Starting RTSP stream for topic: %s", topic_.c_str());

  // Start the RTSP server in a separate thread
  std::thread([this]() {
    try {
      rtsp_server_->run();
    } catch (const std::exception& e) {
      RCLCPP_ERROR(node_->get_logger(), "RTSP server error: %s", e.what());
    }
  }).detach();

  // Subscribe to ROS topic
  std::string subscriber_type = "image";  // Default to image

  // Try to determine subscriber type based on topic
  auto tnat = node_->get_topic_names_and_types();
  for (const auto& topic_and_types : tnat) {
    if (topic_and_types.first == topic_) {
      if (!topic_and_types.second.empty()) {
        const std::string& topic_type = topic_and_types.second[0];
        if (topic_type == "sensor_msgs/msg/PointCloud2") {
          subscriber_type = "pointcloud2";
        }
      }
      break;
    }
  }

  if (subscriber_types_.find(subscriber_type) != subscriber_types_.end()) {
    subscriber_ = subscriber_types_[subscriber_type]->create_subscriber(node_);

    // Create a dummy HTTP request for subscriber configuration
    async_web_server_cpp::HttpRequest dummy_request;
    dummy_request.query = "qos_profile=default";

    subscriber_->subscribe(
      dummy_request,
      topic_,
      std::bind(&RTSPStreamer::imageCallback, this, std::placeholders::_1)
    );
  }

  RCLCPP_INFO(node_->get_logger(), "RTSP stream started on port %d for topic %s", rtsp_port_, topic_.c_str());

  active_ = true;
  streaming_ = true;
}

void RTSPStreamer::switchTopic(const std::string& new_topic)
{
  if (topic_ == new_topic) {
    return; // No change needed
  }
  
  RCLCPP_INFO(node_->get_logger(), "Switching RTSP stream from topic %s to %s", topic_.c_str(), new_topic.c_str());
  
  // Update topic
  topic_ = new_topic;
  
  // Reset subscriber
  subscriber_.reset();
  
  // Create new media stream for the new topic
  auto media_stream = std::make_shared<rtsp_async_server_cpp::H264MediaStream>(
    topic_, width_, height_, fps_, bitrate_);
  rtsp_server_->removeMediaStream(topic_);
  rtsp_server_->addMediaStream(topic_, media_stream);
  
  // Determine new subscriber type
  std::string subscriber_type = "image";  // Default to image
  
  // Try to determine subscriber type based on topic
  auto tnat = node_->get_topic_names_and_types();
  for (const auto& topic_and_types : tnat) {
    if (topic_and_types.first == topic_) {
      if (!topic_and_types.second.empty()) {
        const std::string& topic_type = topic_and_types.second[0];
        if (topic_type == "sensor_msgs/msg/PointCloud2") {
          subscriber_type = "pointcloud2";
        }
      }
      break;
    }
  }
  
  // Create new subscriber for the new topic
  if (subscriber_types_.find(subscriber_type) != subscriber_types_.end()) {
    subscriber_ = subscriber_types_[subscriber_type]->create_subscriber(node_);
    
    // Create a dummy HTTP request for subscriber configuration
    async_web_server_cpp::HttpRequest dummy_request;
    dummy_request.query = "qos_profile=default";
    
    subscriber_->subscribe(
      dummy_request,
      topic_,
      std::bind(&RTSPStreamer::imageCallback, this, std::placeholders::_1)
    );
  }
  
  RCLCPP_INFO(node_->get_logger(), "RTSP stream switched to topic %s", topic_.c_str());
}

void RTSPStreamer::stop()
{
  if (!active_) {
    return;
  }

  RCLCPP_INFO(node_->get_logger(), "Stopping RTSP stream for topic: %s", topic_.c_str());

  active_ = false;
  streaming_ = false;

  // Stop the RTSP server
  if (rtsp_server_) {
    rtsp_server_->stop();
  }

  // Reset subscriber
  subscriber_.reset();
}

std::string RTSPStreamer::getStreamUrl() const
{
  return "rtsp://localhost:" + std::to_string(rtsp_port_) + "/" + topic_;
}

bool RTSPStreamer::handleRTSPRequest(
  const rtsp_async_server_cpp::RTSPRequest& request,
  std::shared_ptr<rtsp_async_server_cpp::RTSPConnection> connection)
{
  // The rtsp_async_server_cpp library handles RTSP requests internally
  // This is just a placeholder for custom request handling if needed
  RCLCPP_DEBUG(node_->get_logger(), "RTSP request: %s %s", 
               rtsp_async_server_cpp::RTSPRequest::methodToString(request.getMethod()).c_str(),
               request.getUri().c_str());
  
  // Return false to let the library handle the request
  return false;
}

void RTSPStreamer::imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
{
  if (!streaming_) {
    return;
  }

  cv_bridge::CvImagePtr cv_ptr;
  try {
    // Convert to BGR8
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  } catch (cv_bridge::Exception& e) {
    RCLCPP_ERROR(node_->get_logger(), "cv_bridge exception: %s", e.what());
    return;
  }

  cv::Mat frame = cv_ptr->image;

  // Get media stream
  auto media_stream = rtsp_server_->getMediaStream(topic_);
  if (!media_stream) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
                        "Media stream not found for topic: %s", topic_.c_str());
    return;
  }

  // Update media stream parameters if frame size changed
  auto h264_stream = std::dynamic_pointer_cast<rtsp_async_server_cpp::H264MediaStream>(media_stream);
  if (h264_stream) {
    if (frame.cols != width_ || frame.rows != height_) {
      width_ = frame.cols;
      height_ = frame.rows;
      h264_stream->setVideoParameters(width_, height_, fps_, bitrate_);
    }

    // Process frame data
    h264_stream->processFrame(frame.data, frame.total() * frame.elemSize(), getTimeStamp());
  }
}

uint32_t RTSPStreamer::getTimeStamp()
{
  auto now = std::chrono::steady_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(now - start_time_);
  return static_cast<uint32_t>(duration.count() * 90); // Convert to 90kHz clock
}

// RTSPStreamerManager implementation

RTSPStreamerManager::RTSPStreamerManager(rclcpp::Node::SharedPtr node)
: node_(node), next_port_(8554)
{
  // Start cleanup timer
  cleanup_timer_ = node_->create_wall_timer(
    5s, std::bind(&RTSPStreamerManager::cleanupInactiveStreamers, this));
}

RTSPStreamerManager::~RTSPStreamerManager()
{
  cleanup();
}

std::shared_ptr<RTSPStreamer> RTSPStreamerManager::createStreamer(
  const std::string & topic,
  const std::string & codec,
  int rtsp_port)
{
  std::lock_guard<std::mutex> lock(streamers_mutex_);
  
  // Create a unique stream key combining topic and codec for better identification
  std::string stream_key = topic + "_" + codec;
  
  // Check if streamer already exists for this exact topic
  auto it = streamers_.find(stream_key);
  if (it != streamers_.end() && it->second->isActive()) {
    RCLCPP_INFO(node_->get_logger(), "Reusing existing RTSP stream for %s", stream_key.c_str());
    return it->second;
  }
  
  // Assign port if not specified
  if (rtsp_port == 0) {
    rtsp_port = next_port_++;
  }
  
  // Create new streamer
  RCLCPP_INFO(node_->get_logger(), "Creating new RTSP stream for %s on port %d", stream_key.c_str(), rtsp_port);
  auto streamer = std::make_shared<RTSPStreamer>(node_, topic, codec, rtsp_port);
  streamers_[stream_key] = streamer;
  
  return streamer;
}

void RTSPStreamerManager::removeStreamer(const std::string & topic)
{
  std::lock_guard<std::mutex> lock(streamers_mutex_);
  
  // Remove all streams for this topic (any codec)
  auto it = streamers_.begin();
  while (it != streamers_.end()) {
    if (it->first.find(topic + "_") == 0) {
      RCLCPP_INFO(node_->get_logger(), "Removing RTSP stream for %s", it->first.c_str());
      it->second->stop();
      it = streamers_.erase(it);
    } else {
      ++it;
    }
  }
}

std::shared_ptr<RTSPStreamer> RTSPStreamerManager::getStreamer(const std::string & topic)
{
  std::lock_guard<std::mutex> lock(streamers_mutex_);
  
  // Search for any stream matching the topic (first found)
  for (const auto& pair : streamers_) {
    if (pair.first.find(topic + "_") == 0) {
      return pair.second;
    }
  }
  
  return nullptr;
}

std::vector<std::string> RTSPStreamerManager::getActiveStreams() const
{
  std::lock_guard<std::mutex> lock(streamers_mutex_);
  std::vector<std::string> active_streams;
  
  for (const auto& pair : streamers_) {
    if (pair.second->isActive()) {
      active_streams.push_back(pair.first);
    }
  }
  
  return active_streams;
}

void RTSPStreamerManager::cleanup()
{
  std::lock_guard<std::mutex> lock(streamers_mutex_);
  for (auto& pair : streamers_) {
    pair.second->stop();
  }
  streamers_.clear();
}

void RTSPStreamerManager::cleanupInactiveStreamers()
{
  std::lock_guard<std::mutex> lock(streamers_mutex_);
  
  auto it = streamers_.begin();
  while (it != streamers_.end()) {
    if (!it->second->isActive()) {
      it = streamers_.erase(it);
    } else {
      ++it;
    }
  }
}

}  // namespace web_video_server
