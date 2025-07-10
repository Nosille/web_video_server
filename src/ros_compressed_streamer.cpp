// Copyright (c) 2014, Worcester Polytechnic Institute
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

#include "web_video_server/ros_compressed_streamer.hpp"
#include "async_web_server_cpp/http_reply.hpp"

#ifdef CV_BRIDGE_USES_OLD_HEADERS
#include "cv_bridge/cv_bridge.h"
#else
#include "cv_bridge/cv_bridge.hpp"
#endif

namespace web_video_server
{

RosCompressedStreamer::RosCompressedStreamer(
  const async_web_server_cpp::HttpRequest & request,
  async_web_server_cpp::HttpConnectionPtr connection, rclcpp::Node::SharedPtr node)
: ImageStreamer(request, connection, node), stream_(connection)
{
  stream_.sendInitialHeader();
}

RosCompressedStreamer::~RosCompressedStreamer()
{
  this->inactive_ = true;
  std::scoped_lock lock(send_mutex_);  // protects sendImage.
}

void RosCompressedStreamer::restreamFrame(std::chrono::duration<double> max_age)
{
  if (inactive_ || (last_msg == 0)) {
    return;
  }

  if (last_frame_ + max_age < std::chrono::steady_clock::now()) {
    std::scoped_lock lock(send_mutex_);
    // don't update last_frame, it may remain an old value.
    sendImage(last_msg, std::chrono::steady_clock::now());
  }
}

void RosCompressedStreamer::sendImage(const cv::Mat & img, const std::chrono::steady_clock::time_point & time)
{
  // std::vector<int> encode_params;
  // encode_params.push_back(cv::IMWRITE_PNG_COMPRESSION);
  // encode_params.push_back(quality_);

  // std::vector<uchar> encoded_buffer;
  // cv::imencode(".png", img, encoded_buffer, encode_params);

  // stream_.sendPartAndClear(time, "image/png", encoded_buffer);
}

void RosCompressedStreamer::sendImage(
  const sensor_msgs::msg::CompressedImage::ConstSharedPtr msg,
  const std::chrono::steady_clock::time_point & time)
{
  try {
    std::string content_type;
    if (msg->format.find("jpeg") != std::string::npos ||
      msg->format.find("jpg") != std::string::npos)
    {
      content_type = "image/jpeg";
    } else if (msg->format.find("png") != std::string::npos) {
      content_type = "image/png";
    } else {
      RCLCPP_WARN(
        node_->get_logger(), "Unknown ROS compressed image format: %s",
        msg->format.c_str());
      return;
    }

    stream_.sendPart(time, content_type, boost::asio::buffer(msg->data), msg);
  } catch (boost::system::system_error & e) {
    // happens when client disconnects
    RCLCPP_DEBUG(node_->get_logger(), "system_error exception: %s", e.what());
    inactive_ = true;
    return;
  } catch (std::exception & e) {
    auto & clk = *node_->get_clock();
    RCLCPP_ERROR_THROTTLE(node_->get_logger(), clk, 40, "exception: %s", e.what());
    inactive_ = true;
    return;
  } catch (...) {
    auto & clk = *node_->get_clock();
    RCLCPP_ERROR_THROTTLE(node_->get_logger(), clk, 40, "exception");
    inactive_ = true;
    return;
  }
}


void RosCompressedStreamer::compressedImageCallback(
  const sensor_msgs::msg::CompressedImage::ConstPtr msg)
{
  std::scoped_lock lock(send_mutex_);  // protects last_msg and last_frame
  last_msg = msg;
  last_frame_ = std::chrono::steady_clock::now();
  sendImage(last_msg, last_frame_);
}


std::shared_ptr<ImageStreamer> RosCompressedStreamerType::create_streamer(
  const async_web_server_cpp::HttpRequest & request,
  async_web_server_cpp::HttpConnectionPtr connection,
  rclcpp::Node::SharedPtr node)
{
  return std::make_shared<RosCompressedStreamer>(request, connection, node);
}

std::string RosCompressedStreamerType::create_viewer(
  const async_web_server_cpp::HttpRequest & request)
{
  std::stringstream ss;
  ss << "<img src=\"/stream?";
  ss << request.query;
  ss << "\"></img>";
  return ss.str();
}

}  // namespace web_video_server
