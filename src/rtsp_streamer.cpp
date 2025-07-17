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
//      documentation and/or other materials provided with the distribution.
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

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <fcntl.h>
#include <random>
#include <sstream>
#include <algorithm>
#include <cstring>

#ifdef CV_BRIDGE_USES_OLD_HEADERS
#include "cv_bridge/cv_bridge.h"
#else
#include "cv_bridge/cv_bridge.hpp"
#endif

#include "sensor_msgs/image_encodings.hpp"

using namespace std::chrono_literals;

namespace web_video_server
{

RTSPStreamer::RTSPStreamer(
  rclcpp::Node::SharedPtr node,
  const std::string & topic,
  const std::string & codec_name,
  int rtsp_port)
: node_(node),
  topic_(topic),
  codec_name_(codec_name),
  rtsp_port_(rtsp_port),
  server_socket_(-1),
  active_(false),
  streaming_(false),
  format_context_(nullptr),
  codec_(nullptr),
  codec_context_(nullptr),
  video_stream_(nullptr),
  frame_(nullptr),
  packet_(nullptr),
  sws_context_(nullptr),
  width_(640),
  height_(480),
  fps_(30),
  bitrate_(1000000),
  rtp_timestamp_(0),
  rtp_sequence_(0),
  rtp_ssrc_(0)
{
  // Initialize subscriber types
  subscriber_types_["image"] = std::make_shared<ImageTransportSubscriberType>();
  subscriber_types_["pointcloud2"] = std::make_shared<PointCloud2SubscriberType>();
  
  // Generate random SSRC for RTP
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_int_distribution<uint32_t> dis(0, UINT32_MAX);
  rtp_ssrc_ = dis(gen);
  
  start_time_ = std::chrono::steady_clock::now();
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
  
  // Create server socket
  server_socket_ = socket(AF_INET, SOCK_STREAM, 0);
  if (server_socket_ < 0) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to create RTSP server socket");
    return;
  }
  
  // Set socket options
  int opt = 1;
  if (setsockopt(server_socket_, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt)) < 0) {
    RCLCPP_WARN(node_->get_logger(), "Failed to set socket options");
  }
  
  // Bind socket
  struct sockaddr_in server_addr;
  server_addr.sin_family = AF_INET;
  server_addr.sin_addr.s_addr = INADDR_ANY;
  server_addr.sin_port = htons(rtsp_port_);
  
  if (bind(server_socket_, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to bind RTSP server socket to port %d", rtsp_port_);
    close(server_socket_);
    return;
  }
  
  // Listen for connections
  if (listen(server_socket_, 5) < 0) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to listen on RTSP server socket");
    close(server_socket_);
    return;
  }
  
  active_ = true;
  
  // Start RTSP server thread
  server_thread_ = std::thread(&RTSPStreamer::rtspServerThread, this);
  
  // Start RTP streaming thread
  rtp_thread_ = std::thread(&RTSPStreamer::rtpStreamThread, this);
  
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
}

void RTSPStreamer::stop()
{
  if (!active_) {
    return;
  }
  
  RCLCPP_INFO(node_->get_logger(), "Stopping RTSP stream for topic: %s", topic_.c_str());
  
  active_ = false;
  streaming_ = false;
  
  // Close server socket
  if (server_socket_ >= 0) {
    close(server_socket_);
    server_socket_ = -1;
  }
  
  // Close all client connections
  {
    std::lock_guard<std::mutex> lock(clients_mutex_);
    for (auto& client_pair : clients_) {
      client_pair.second->active = false;
      if (client_pair.second->socket_fd >= 0) {
        close(client_pair.second->socket_fd);
      }
    }
    clients_.clear();
  }
  
  // Join threads
  if (server_thread_.joinable()) {
    server_thread_.join();
  }
  
  if (rtp_thread_.joinable()) {
    frame_cv_.notify_all();
    rtp_thread_.join();
  }
  
  // Reset subscriber
  subscriber_.reset();
  
  // Cleanup encoder
  cleanupEncoder();
}

std::string RTSPStreamer::getStreamUrl() const
{
  return "rtsp://localhost:" + std::to_string(rtsp_port_) + "/" + topic_;
}

void RTSPStreamer::rtspServerThread()
{
  while (active_) {
    struct sockaddr_in client_addr;
    socklen_t client_len = sizeof(client_addr);
    
    int client_socket = accept(server_socket_, (struct sockaddr*)&client_addr, &client_len);
    if (client_socket < 0) {
      if (active_) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to accept RTSP client connection");
      }
      continue;
    }
    
    // Handle client in separate thread
    std::thread([this, client_socket]() {
      handleRTSPRequest(client_socket);
    }).detach();
  }
}

void RTSPStreamer::handleRTSPRequest(int client_socket)
{
  char buffer[4096];
  while (active_) {
    ssize_t received = recv(client_socket, buffer, sizeof(buffer) - 1, 0);
    if (received <= 0) {
      break;
    }
    
    buffer[received] = '\0';
    std::string request(buffer);
    
    // Parse RTSP request
    std::istringstream iss(request);
    std::string method, uri, version;
    iss >> method >> uri >> version;
    
    RCLCPP_DEBUG(node_->get_logger(), "RTSP %s request for %s", method.c_str(), uri.c_str());
    
    if (method == "DESCRIBE") {
      handleDescribe(client_socket, uri);
    } else if (method == "SETUP") {
      // Extract transport from request
      std::string transport;
      size_t transport_pos = request.find("Transport:");
      if (transport_pos != std::string::npos) {
        size_t line_end = request.find("\r\n", transport_pos);
        transport = request.substr(transport_pos + 10, line_end - transport_pos - 10);
        // Trim whitespace
        transport.erase(0, transport.find_first_not_of(" \t"));
        transport.erase(transport.find_last_not_of(" \t") + 1);
      }
      handleSetup(client_socket, transport);
    } else if (method == "PLAY") {
      // Extract session from request
      std::string session;
      size_t session_pos = request.find("Session:");
      if (session_pos != std::string::npos) {
        size_t line_end = request.find("\r\n", session_pos);
        session = request.substr(session_pos + 8, line_end - session_pos - 8);
        // Trim whitespace
        session.erase(0, session.find_first_not_of(" \t"));
        session.erase(session.find_last_not_of(" \t") + 1);
      }
      handlePlay(client_socket, session);
    } else if (method == "TEARDOWN") {
      // Extract session from request
      std::string session;
      size_t session_pos = request.find("Session:");
      if (session_pos != std::string::npos) {
        size_t line_end = request.find("\r\n", session_pos);
        session = request.substr(session_pos + 8, line_end - session_pos - 8);
        // Trim whitespace
        session.erase(0, session.find_first_not_of(" \t"));
        session.erase(session.find_last_not_of(" \t") + 1);
      }
      handleTeardown(client_socket, session);
      break;
    }
  }
  
  close(client_socket);
}

void RTSPStreamer::handleDescribe(int client_socket, const std::string& uri)
{
  std::string sdp = generateSDPDescription();
  
  std::stringstream response;
  response << "RTSP/1.0 200 OK\r\n"
           << "Content-Type: application/sdp\r\n"
           << "Content-Length: " << sdp.length() << "\r\n"
           << "\r\n"
           << sdp;
  
  send(client_socket, response.str().c_str(), response.str().length(), 0);
}

void RTSPStreamer::handleSetup(int client_socket, const std::string& transport)
{
  std::string session_id = generateSessionId();
  
  // Parse transport info to get client RTP ports
  uint16_t client_rtp_port = 0;
  uint16_t client_rtcp_port = 0;
  
  size_t client_port_pos = transport.find("client_port=");
  if (client_port_pos != std::string::npos) {
    std::string port_range = transport.substr(client_port_pos + 12);
    size_t dash_pos = port_range.find('-');
    if (dash_pos != std::string::npos) {
      client_rtp_port = std::stoi(port_range.substr(0, dash_pos));
      client_rtcp_port = std::stoi(port_range.substr(dash_pos + 1));
    }
  }
  
  // Create client record
  auto client = std::make_shared<RTSPClient>();
  client->socket_fd = client_socket;
  client->session_id = session_id;
  client->transport_info = transport;
  client->rtp_port = client_rtp_port;
  client->rtcp_port = client_rtcp_port;
  
  // Get client IP
  struct sockaddr_in client_addr;
  socklen_t addr_len = sizeof(client_addr);
  if (getpeername(client_socket, (struct sockaddr*)&client_addr, &addr_len) == 0) {
    client->client_ip = inet_ntoa(client_addr.sin_addr);
  }
  
  {
    std::lock_guard<std::mutex> lock(clients_mutex_);
    clients_[session_id] = client;
  }
  
  std::stringstream response;
  response << "RTSP/1.0 200 OK\r\n"
           << "Transport: " << transport << ";server_port=" << rtsp_port_ << "-" << (rtsp_port_ + 1) << "\r\n"
           << "Session: " << session_id << "\r\n"
           << "\r\n";
  
  send(client_socket, response.str().c_str(), response.str().length(), 0);
}

void RTSPStreamer::handlePlay(int client_socket, const std::string& session)
{
  std::stringstream response;
  response << "RTSP/1.0 200 OK\r\n"
           << "Session: " << session << "\r\n"
           << "\r\n";
  
  send(client_socket, response.str().c_str(), response.str().length(), 0);
  
  streaming_ = true;
  
  // Initialize encoder if not already done
  if (!codec_context_) {
    initializeEncoder();
  }
}

void RTSPStreamer::handleTeardown(int client_socket, const std::string& session)
{
  std::stringstream response;
  response << "RTSP/1.0 200 OK\r\n"
           << "Session: " << session << "\r\n"
           << "\r\n";
  
  send(client_socket, response.str().c_str(), response.str().length(), 0);
  
  // Remove client
  {
    std::lock_guard<std::mutex> lock(clients_mutex_);
    clients_.erase(session);
  }
  
  // Stop streaming if no clients
  if (clients_.empty()) {
    streaming_ = false;
  }
}

void RTSPStreamer::rtpStreamThread()
{
  while (active_) {
    std::unique_lock<std::mutex> lock(frame_mutex_);
    frame_cv_.wait(lock, [this] { return !frame_queue_.empty() || !active_; });
    
    if (!active_) {
      break;
    }
    
    if (streaming_ && !frame_queue_.empty()) {
      cv::Mat frame = frame_queue_.front();
      frame_queue_.pop();
      lock.unlock();
      
      encodeAndSendFrame(frame);
    }
  }
}

void RTSPStreamer::imageCallback(const sensor_msgs::msg::Image::ConstPtr & msg)
{
  if (!streaming_) {
    return;
  }
  
  cv_bridge::CvImagePtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  } catch (cv_bridge::Exception& e) {
    RCLCPP_ERROR(node_->get_logger(), "cv_bridge exception: %s", e.what());
    return;
  }
  
  cv::Mat frame = cv_ptr->image;
  
  // Update dimensions if needed
  if (frame.cols != width_ || frame.rows != height_) {
    width_ = frame.cols;
    height_ = frame.rows;
    
    // Reinitialize encoder with new dimensions
    cleanupEncoder();
    initializeEncoder();
  }
  
  // Add frame to queue
  {
    std::lock_guard<std::mutex> lock(frame_mutex_);
    frame_queue_.push(frame);
    
    // Keep queue size reasonable
    while (frame_queue_.size() > 5) {
      frame_queue_.pop();
    }
  }
  
  frame_cv_.notify_one();
}

void RTSPStreamer::encodeAndSendFrame(const cv::Mat & frame)
{
  std::lock_guard<std::mutex> lock(encode_mutex_);
  
  if (!codec_context_ || !frame_) {
    return;
  }
  
  // Convert frame to AVFrame
  AVFrame* input_frame = av_frame_alloc();
  av_image_fill_arrays(
    input_frame->data, input_frame->linesize,
    frame.data, AV_PIX_FMT_BGR24, width_, height_, 1);
  
  // Convert color space
  if (!sws_context_) {
    sws_context_ = sws_getContext(
      width_, height_, AV_PIX_FMT_BGR24,
      width_, height_, AV_PIX_FMT_YUV420P,
      SWS_BILINEAR, nullptr, nullptr, nullptr);
  }
  
  sws_scale(
    sws_context_,
    (const uint8_t * const *)input_frame->data, input_frame->linesize,
    0, height_,
    frame_->data, frame_->linesize);
  
  av_frame_free(&input_frame);
  
  // Set frame properties
  frame_->pts = rtp_timestamp_;
  
  // Encode frame
  int ret = avcodec_send_frame(codec_context_, frame_);
  if (ret < 0) {
    RCLCPP_ERROR(node_->get_logger(), "Error sending frame to encoder");
    return;
  }
  
  while (ret >= 0) {
    ret = avcodec_receive_packet(codec_context_, packet_);
    if (ret == AVERROR(EAGAIN) || ret == AVERROR_EOF) {
      break;
    } else if (ret < 0) {
      RCLCPP_ERROR(node_->get_logger(), "Error receiving packet from encoder");
      break;
    }
    
    // Send RTP packet to all clients
    {
      std::lock_guard<std::mutex> lock(clients_mutex_);
      for (auto& client_pair : clients_) {
        auto& client = client_pair.second;
        if (client->active && client->rtp_port > 0) {
          // Create RTP packet (simplified)
          // In a real implementation, you'd need proper RTP packetization
          // For now, we'll just send the raw H.264 data
          
          struct sockaddr_in client_addr;
          client_addr.sin_family = AF_INET;
          client_addr.sin_port = htons(client->rtp_port);
          inet_pton(AF_INET, client->client_ip.c_str(), &client_addr.sin_addr);
          
          int rtp_socket = socket(AF_INET, SOCK_DGRAM, 0);
          if (rtp_socket >= 0) {
            sendto(rtp_socket, packet_->data, packet_->size, 0,
                   (struct sockaddr*)&client_addr, sizeof(client_addr));
            close(rtp_socket);
          }
        }
      }
    }
    
    rtp_timestamp_ += 3000; // Increment timestamp
    rtp_sequence_++;
    
    av_packet_unref(packet_);
  }
}

void RTSPStreamer::initializeEncoder()
{
  // Find encoder
  codec_ = avcodec_find_encoder_by_name(codec_name_.c_str());
  if (!codec_) {
    RCLCPP_ERROR(node_->get_logger(), "Codec '%s' not found", codec_name_.c_str());
    return;
  }
  
  // Allocate codec context
  codec_context_ = avcodec_alloc_context3(codec_);
  if (!codec_context_) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to allocate codec context");
    return;
  }
  
  // Set codec parameters
  codec_context_->bit_rate = bitrate_;
  codec_context_->width = width_;
  codec_context_->height = height_;
  codec_context_->time_base = {1, fps_};
  codec_context_->framerate = {fps_, 1};
  codec_context_->gop_size = 10;
  codec_context_->max_b_frames = 1;
  codec_context_->pix_fmt = AV_PIX_FMT_YUV420P;
  
  // Set codec options for real-time streaming
  if (codec_->id == AV_CODEC_ID_H264) {
    av_opt_set(codec_context_->priv_data, "preset", "ultrafast", 0);
    av_opt_set(codec_context_->priv_data, "tune", "zerolatency", 0);
  }
  
  // Open codec
  if (avcodec_open2(codec_context_, codec_, nullptr) < 0) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to open codec");
    avcodec_free_context(&codec_context_);
    return;
  }
  
  // Allocate frame
  frame_ = av_frame_alloc();
  if (!frame_) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to allocate frame");
    return;
  }
  
  frame_->format = codec_context_->pix_fmt;
  frame_->width = width_;
  frame_->height = height_;
  
  if (av_frame_get_buffer(frame_, 0) < 0) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to allocate frame buffer");
    return;
  }
  
  // Allocate packet
  packet_ = av_packet_alloc();
  if (!packet_) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to allocate packet");
    return;
  }
}

void RTSPStreamer::cleanupEncoder()
{
  if (packet_) {
    av_packet_free(&packet_);
    packet_ = nullptr;
  }
  
  if (frame_) {
    av_frame_free(&frame_);
    frame_ = nullptr;
  }
  
  if (codec_context_) {
    avcodec_free_context(&codec_context_);
    codec_context_ = nullptr;
  }
  
  if (sws_context_) {
    sws_freeContext(sws_context_);
    sws_context_ = nullptr;
  }
}

std::string RTSPStreamer::generateSessionId()
{
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_int_distribution<> dis(0, 15);
  
  std::string session_id;
  for (int i = 0; i < 8; ++i) {
    session_id += "0123456789ABCDEF"[dis(gen)];
  }
  
  return session_id;
}

std::string RTSPStreamer::generateSDPDescription()
{
  std::stringstream sdp;
  
  sdp << "v=0\r\n"
      << "o=- 0 0 IN IP4 127.0.0.1\r\n"
      << "s=ROS Video Stream\r\n"
      << "c=IN IP4 0.0.0.0\r\n"
      << "t=0 0\r\n"
      << "a=tool:web_video_server\r\n"
      << "m=video 0 RTP/AVP 96\r\n"
      << "a=rtpmap:96 H264/90000\r\n"
      << "a=fmtp:96 packetization-mode=1\r\n"
      << "a=control:*\r\n";
  
  return sdp.str();
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
  
  // Check if streamer already exists
  auto it = streamers_.find(topic);
  if (it != streamers_.end() && it->second->isActive()) {
    return it->second;
  }
  
  // Assign port if not specified
  if (rtsp_port == 0) {
    rtsp_port = next_port_++;
  }
  
  auto streamer = std::make_shared<RTSPStreamer>(node_, topic, codec, rtsp_port);
  streamers_[topic] = streamer;
  
  return streamer;
}

void RTSPStreamerManager::removeStreamer(const std::string & topic)
{
  std::lock_guard<std::mutex> lock(streamers_mutex_);
  auto it = streamers_.find(topic);
  if (it != streamers_.end()) {
    it->second->stop();
    streamers_.erase(it);
  }
}

std::shared_ptr<RTSPStreamer> RTSPStreamerManager::getStreamer(const std::string & topic)
{
  std::lock_guard<std::mutex> lock(streamers_mutex_);
  auto it = streamers_.find(topic);
  return (it != streamers_.end()) ? it->second : nullptr;
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

// PointCloud2SubscriberType is defined in the header file

}  // namespace web_video_server
