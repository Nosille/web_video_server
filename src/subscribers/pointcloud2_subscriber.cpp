// Copyright (c) 2024-2025, The Robot Web Tools Contributors
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

#include <algorithm>
#include <chrono>
#include <cstring>
#include <limits>

#include "rclcpp/node.hpp"
#include "rclcpp/logging.hpp"
#include "rmw/qos_profiles.h"

#include "async_web_server_cpp/http_request.hpp"
#include "web_video_server/subscribers/pointcloud2_subscriber.hpp"
#include "web_video_server/utils.hpp"
#include "web_video_server/subscriber.hpp"

namespace web_video_server
{
namespace subscribers
{
PointCloud2Subscriber::PointCloud2Subscriber(rclcpp::Node::WeakPtr _node)
: SubscriberBase(_node, "pointcloud2_subscriber")
{
  auto node = lock_node();
  if (!node) {
    inactive_ = true;
    return;
  }

  std::scoped_lock lock(subscriber_mutex_);

  if (!node->has_parameter("frame_id")) node->declare_parameter("frame_id", "base_link");
  if (!node->has_parameter("wait_for_tf_delay")) node->declare_parameter("wait_for_tf_delay", 0.1);
  if (!node->has_parameter("colorize")) node->declare_parameter("colorize", true);
  if (!node->has_parameter("normalize")) node->declare_parameter("normalize", true);
  if (!node->has_parameter("field")) node->declare_parameter("field", "depth");

  // Initialize our TF items
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node->get_clock(), std::chrono::seconds(10));
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_, node, true);
  
  tf2::Quaternion q;
  q.setRPY(M_PI / 2.0, - M_PI / 2.0, 0.0);
  transform_optical_.transform.translation.x = 0.0;
  transform_optical_.transform.translation.y = 0.0;
  transform_optical_.transform.translation.z = 0.0;
  transform_optical_.transform.rotation.x = q.x();
  transform_optical_.transform.rotation.y = q.y();
  transform_optical_.transform.rotation.z = q.z();
  transform_optical_.transform.rotation.w = q.w();
}

PointCloud2Subscriber::~PointCloud2Subscriber()
{
  std::scoped_lock lock(subscriber_mutex_);
  inactive_ = true;
}

void PointCloud2Subscriber::subscribe(const async_web_server_cpp::HttpRequest &request,
                                         const std::string& topic, 
                                         const ImageCallback& callback)
{
  auto node = lock_node();
  if (!node) {
    inactive_ = true;
    return;
  }
  
  std::scoped_lock lock(subscriber_mutex_);

  callback_ = callback;
  std::string default_qos_profile = node->get_parameter("default_qos_profile").as_string();    
  auto qos_profile_name = request.get_query_param_value_or_default("qos_profile", default_qos_profile);

  wait_for_tf_delay_ = node->get_parameter("wait_for_tf_delay").as_double();
  wait_for_tf_delay_ = request.get_query_param_value_or_default("wait_for_tf_delay", wait_for_tf_delay_);
  
  
  std::string default_frame_id = node->get_parameter("frame_id").as_string();
  frame_id_ = request.get_query_param_value_or_default("frame_id", default_frame_id);
  
  bool default_color = node->get_parameter("colorize").as_bool();
  colorize_ = request.get_query_param_value_or_default<bool>("colorize", default_color); 

  bool default_normalize = node->get_parameter("normalize").as_bool();
  normalize_ = request.get_query_param_value_or_default<bool>("normalize", default_normalize);

  std::string default_field = node->get_parameter("field").as_string();
  field_ = request.get_query_param_value_or_default("field", default_field);
  
  height_ = request.get_query_param_value_or_default<int>("height", 600);
  width_  = request.get_query_param_value_or_default<int>("width", 800);
  pixel_size_ = request.get_query_param_value_or_default<int>("pixel_size", 5);
  focal_length_ = request.get_query_param_value_or_default<double>("focal_length", height_/2.0);  

  // Get QoS profile from query parameter
  RCLCPP_INFO(
    logger_, "Streaming topic %s with QoS profile %s", topic.c_str(),
    qos_profile_name.c_str());
  auto qos_profile = get_qos_profile_from_name(qos_profile_name);
  if (!qos_profile) {
    qos_profile = rmw_qos_profile_default;
    RCLCPP_ERROR(
      logger_,
      "Invalid QoS profile %s specified. Using default profile.",
      qos_profile_name.c_str());
  }

  const auto qos = rclcpp::QoS(
    rclcpp::QoSInitialization(qos_profile.value().history, 1),
    qos_profile.value());

  cbg_ = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);    
  rclcpp::SubscriptionOptions options;
  options.callback_group = cbg_;  
  
  sub_ = node->create_subscription<sensor_msgs::msg::PointCloud2>(
    topic, qos, std::bind(&PointCloud2Subscriber::subscriber_callback, this, std::placeholders::_1), options
  );
}

void PointCloud2Subscriber::subscriber_callback(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr &input_msg)
{
  std::scoped_lock lock(subscriber_mutex_);

  if(inactive_) return;

  if (input_msg->data.size() == 0)
  {
    RCLCPP_WARN_STREAM_THROTTLE(logger_, *clock_, 1000, "No data in pointcloud!");
    return;
  }

  try
  {
    ProcessCloud(input_msg);
  }
  catch (const cv::Exception &e)
  {
    RCLCPP_WARN_STREAM_THROTTLE(logger_, *clock_, 1000, "OpenCV exception while rendering pointcloud: " << e.what());
  }
  catch (const std::exception &e)
  {
    RCLCPP_WARN_STREAM_THROTTLE(logger_, *clock_, 1000, "Exception while rendering pointcloud: " << e.what());
  }
}

void PointCloud2Subscriber::ProcessCloud(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr &input_msg)
{
  // Start timer
  auto beginTime = std::chrono::steady_clock::now();

  //transform
  sensor_msgs::msg::PointCloud2 output_cloud; 
  output_cloud = TransformFrame(input_msg, frame_id_);

  // Check for malformed pointclouds
  if (output_cloud.point_step == 0 || output_cloud.width == 0 || output_cloud.height == 0)
  {
    RCLCPP_WARN_STREAM_THROTTLE(logger_, *clock_, 1000, "Malformed pointcloud: zero dimensions or point_step!");
    return;
  }  
  else if (output_cloud.data.size() < output_cloud.height * output_cloud.width * output_cloud.point_step) {
    RCLCPP_WARN_STREAM_THROTTLE(logger_, *clock_, 1000, "Malformed pointcloud: data buffer smaller than declared dimensions!");
    return;
  }

  // Find relevant fields
  sensor_msgs::msg::PointField xField, yField, zField, userField;
  userField.name = field_;
  if(!FindFields(input_msg, userField, xField, yField, zField)) {
    RCLCPP_WARN_STREAM_THROTTLE(logger_, *clock_, 1000, "Required fields not found in pointcloud (field: " << field_ << ")");
    return;
  }
  const size_t user_bytes = UserFieldBytes(userField);
  if (xField.offset + sizeof(float) > output_cloud.point_step ||
      yField.offset + sizeof(float) > output_cloud.point_step ||
      zField.offset + sizeof(float) > output_cloud.point_step ||
      (user_bytes > 0 && userField.offset + user_bytes > output_cloud.point_step))
  {
    RCLCPP_WARN_STREAM_THROTTLE(logger_, *clock_, 1000, "Malformed pointcloud: field offsets exceed point_step!");
    return;
  }

  // Setup camera_info
  RCLCPP_DEBUG_STREAM(logger_,"    Camera Info");
  cv::Mat intrinsic_matrix, distortion_coefficients;
  GatherCameraInfo(intrinsic_matrix, distortion_coefficients);
  
  // Setup depth image
  RCLCPP_DEBUG_STREAM(logger_,"    Depth");           
  cv_bridge::CvImage depthImage;
  CreateDepthImage(output_cloud.header, depthImage);
  
  // Setup user image
  RCLCPP_DEBUG_STREAM(logger_,"    User-Datatype: " << +userField.datatype);
  cv_bridge::CvImage userImage;
  if(!CreateUserImage(output_cloud.header, userField, userImage)) {
    RCLCPP_WARN_STREAM_THROTTLE(logger_, *clock_, 1000, "Failed to create user image!");    
    return;
  }
  
  // Project Points
   RCLCPP_DEBUG_STREAM(logger_,"  Project points");        
  std::vector<cv::Point3f> obj_pts;
  std::vector<cv::Point2f> img_pts = ProjectPoints(output_cloud, xField, yField, zField, intrinsic_matrix, distortion_coefficients, obj_pts);

  // Process projected points and render to image
  cv::Mat depthMask = cv::Mat::zeros(height_, width_, CV_8UC1);
  // Loop through points and fillout user and depth images
  RCLCPP_DEBUG_STREAM(logger_,"  Process projected points: " << img_pts.size());     
  for (size_t i = 0; i < img_pts.size(); ++i)
  {
    // Check if inflated point is inside field of view.
    // The pixel size parameter requires capturing points that slightly outside FOV.
    // Take care transfering to image since it will crash if the pixels
    // outside FOV are applied to image.
    const float uf = img_pts[i].x;
    const float vf = img_pts[i].y;
    if ((uf >= -pixel_size_) && (uf < width_ + pixel_size_) &&
        (vf >= -pixel_size_) && (vf < height_ + pixel_size_) &&
        obj_pts[i].z > 0.1)  // Filter out points behind/very close to sensor that cause incorrect projection
    {
      int u = int(uf);
      int v = int(vf);
      const size_t point_start = i * output_cloud.point_step;
      // draw box around each pixel based on pixel_size param
      int shift = pixel_size_ / 2;
      int lowerIndex1 = v - shift;
      if(lowerIndex1 < 0) lowerIndex1 = 0;
      int upperIndex1 = v + shift;
      if(upperIndex1 > height_ - 1) upperIndex1 = height_ - 1; 
      int lowerIndex2 = u - shift; 
      if(lowerIndex2 < 0) lowerIndex2 = 0;                           
      int upperIndex2 = u + shift;
      if(upperIndex2 > width_ - 1) upperIndex2 = width_ - 1;
      for(int j = lowerIndex1; j <= upperIndex1; j++)
      {
        for(int k = lowerIndex2; k <= upperIndex2; k++)
        {
          // Update the depth image if point is closer to camera that previous values
          if(depthImage.image.at<float>(j, k) > obj_pts[i].z)
          {
            depthImage.image.at<float>(j, k) = obj_pts[i].z;
            depthMask.at<uint8_t>(j, k) = 255; // 255 = has data, 0 = no data
            
            // user image - write data for the closest point (depth buffer was already updated)
            if( (userField.datatype == sensor_msgs::msg::PointField::UINT8   && userField.count == 4) ||
                (userField.datatype == sensor_msgs::msg::PointField::UINT32  && (userField.name == "rgb" || userField.name == "rgba")) ||
                (userField.datatype == sensor_msgs::msg::PointField::FLOAT32 && (userField.name == "rgb" || userField.name == "rgba"))
              )
            {
              // Handle RGB/RGBA fields - these are packed color data
              uint8_t user1, user2, user3;
              std::memcpy(&user1, &output_cloud.data[point_start + userField.offset + 0], sizeof(uint8_t));
              std::memcpy(&user2, &output_cloud.data[point_start + userField.offset + 1], sizeof(uint8_t));
              std::memcpy(&user3, &output_cloud.data[point_start + userField.offset + 2], sizeof(uint8_t));
              cv::Vec3b pixel;
              pixel[0] = user1;
              pixel[1] = user2;
              pixel[2] = user3;
              userImage.image.at<cv::Vec3b>(j, k) = pixel;
            }
            else if((userField.datatype == sensor_msgs::msg::PointField::UINT16  && userField.count == 4) ||
                    (userField.datatype == sensor_msgs::msg::PointField::FLOAT64 && (userField.name == "rgb" || userField.name == "rgba"))
                    )
            {
              uint16_t user1, user2, user3;
              std::memcpy(&user1, &output_cloud.data[point_start + userField.offset + 0], sizeof(uint16_t));
              std::memcpy(&user2, &output_cloud.data[point_start + userField.offset + 2], sizeof(uint16_t));
              std::memcpy(&user3, &output_cloud.data[point_start + userField.offset + 4], sizeof(uint16_t));
              cv::Vec3w bgr_pixel;
              bgr_pixel[0] = user1;
              bgr_pixel[1] = user2;
              bgr_pixel[2] = user3;
              userImage.image.at<cv::Vec3w>(j, k) = bgr_pixel;
            }
            else if((userField.datatype == sensor_msgs::msg::PointField::UINT8))
            {
              uint8_t value;
              std::memcpy(&value, &output_cloud.data[point_start + userField.offset],sizeof(uint8_t));
              userImage.image.at<uint8_t>(j, k) = value;      
            }
            else if((userField.datatype == sensor_msgs::msg::PointField::INT8))
            {
              int8_t value;
              std::memcpy(&value, &output_cloud.data[point_start + userField.offset],sizeof(int8_t));
              userImage.image.at<int8_t>(j, k) = value;         
            }        
            else if((userField.datatype == sensor_msgs::msg::PointField::UINT16))
            {
              uint16_t value;
              std::memcpy(&value, &output_cloud.data[point_start + userField.offset],sizeof(uint16_t));
              userImage.image.at<uint16_t>(j, k) = value;          
            }
            else if((userField.datatype == sensor_msgs::msg::PointField::INT16))
            {
              int16_t value;
              std::memcpy(&value, &output_cloud.data[point_start + userField.offset],sizeof(int16_t));
              userImage.image.at<int16_t>(j, k) = value;
            }
            else if((userField.datatype == sensor_msgs::msg::PointField::FLOAT32))
            {
              float value;
              std::memcpy(&value, &output_cloud.data[point_start + userField.offset],sizeof(float));
              userImage.image.at<float>(j, k) = value;         
            }        
            else if((userField.datatype == sensor_msgs::msg::PointField::FLOAT64))
            {
              double value;
              std::memcpy(&value, &output_cloud.data[point_start + userField.offset],sizeof(double));
              userImage.image.at<double>(j, k) = value;
            }  
          }   
        } 
      }
    }
  }

  // Pixels never hit by a point still hold max values, so
  // clear them so normalization and non-colorized output stay sane.
  depthImage.image.setTo(0.0f, depthMask == 0);

  // Normalize
  cv_bridge::CvImage normalized_image;
  if (field_ == "depth") {
    if (normalize_ && cv::countNonZero(depthMask) > 0) {
      // Spread the valid depths across the 0..100 range that ConvertToColor
      // maps onto 0..255, instead of the fixed 100 m scale that rendered
      // nearby scenes nearly black.
      double min_v = 0.0, max_v = 0.0;
      cv::minMaxLoc(depthImage.image, &min_v, &max_v, nullptr, nullptr, depthMask);
      normalized_image.header = depthImage.header;
      normalized_image.encoding = depthImage.encoding;
      if (max_v > min_v) {
        normalized_image.image =
          (depthImage.image - static_cast<float>(min_v)) * static_cast<float>(100.0 / (max_v - min_v));
        normalized_image.image.setTo(0.0f, depthMask == 0);
      } else {
        // Single-depth scene: render valid pixels mid-scale
        normalized_image.image = cv::Mat::zeros(height_, width_, CV_32FC1);
        normalized_image.image.setTo(50.0f, depthMask);
      }
    } else {
      normalized_image = depthImage;
    }
  } else if (normalize_) {
    normalized_image = NormalizeImage(userImage);
  } else {
    normalized_image = userImage;
  }

  // Convert to color
  cv_bridge::CvImage colorImage;
  if (colorize_) {
    colorImage = ConvertToColor(depthMask, normalized_image);
  } else {
    colorImage = normalized_image;
  }

  // Performance timing
  auto endTime = std::chrono::steady_clock::now();
  auto totalTime = endTime - beginTime;
  auto timeMS = std::chrono::duration_cast<std::chrono::milliseconds>(totalTime);
  RCLCPP_DEBUG_STREAM(logger_, "Processing time: " << timeMS.count() << "ms");
  sensor_msgs::msg::Image output_msg;
  colorImage.toImageMsg(output_msg);
  sensor_msgs::msg::Image::ConstSharedPtr output_ptr = std::make_shared<sensor_msgs::msg::Image>(output_msg);
  try_forward_image(output_ptr);

  return;
}

bool PointCloud2Subscriber::compareFieldsOffset(sensor_msgs::msg::PointField& field1, sensor_msgs::msg::PointField& field2)
{
  return (field1.offset < field2.offset);
}

size_t PointCloud2Subscriber::UserFieldBytes(const sensor_msgs::msg::PointField &field)
{
  const bool is_color = (field.name == "rgb" || field.name == "rgba");
  if ((field.datatype == sensor_msgs::msg::PointField::UINT8   && field.count == 4) ||
      (field.datatype == sensor_msgs::msg::PointField::UINT32  && is_color) ||
      (field.datatype == sensor_msgs::msg::PointField::FLOAT32 && is_color))
  {
    return 3;
  }
  if ((field.datatype == sensor_msgs::msg::PointField::UINT16  && field.count == 4) ||
      (field.datatype == sensor_msgs::msg::PointField::FLOAT64 && is_color))
  {
    return 6;
  }
  switch (field.datatype)
  {
    case sensor_msgs::msg::PointField::UINT8:
    case sensor_msgs::msg::PointField::INT8:
      return 1;
    case sensor_msgs::msg::PointField::UINT16:
    case sensor_msgs::msg::PointField::INT16:
      return 2;
    case sensor_msgs::msg::PointField::FLOAT32:
      return 4;
    case sensor_msgs::msg::PointField::FLOAT64:
      return 8;
    default:
      return 0;
  }
}

bool PointCloud2Subscriber::FindFields(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &input_msg, sensor_msgs::msg::PointField &userField, 
      sensor_msgs::msg::PointField &xField, sensor_msgs::msg::PointField &yField, sensor_msgs::msg::PointField &zField)
{
  std::vector<sensor_msgs::msg::PointField> sortedFields(input_msg->fields);
  std::sort(sortedFields.begin(), sortedFields.end(), PointCloud2Subscriber::compareFieldsOffset);

  // Find fields we need in the cloud
  bool xFound = false, yFound = false, zFound = false;
  bool userFound = false;
  for (size_t i = 0; i < sortedFields.size(); i++)
  {
    sensor_msgs::msg::PointField currentField = sortedFields[i];
    
    if(currentField.name == "x")
    {
      xFound = true;
      xField = currentField;
    }
    else if(currentField.name == "y")
    {
      yFound = true;
      yField = currentField;
    }
    else if(currentField.name == "z")
    {
      zFound = true;
      zField = currentField;
    }
    if(currentField.name == userField.name)
    {
      userFound = true;
      userField = currentField;
    }
  }

  //Check found fields
  if(!xFound || !yFound || !zFound)
  {
    RCLCPP_WARN_STREAM(logger_,"Cloud does not contain XYZ data!");
    return false;
  }
  else if(xField.datatype != 7 || yField.datatype != 7 || zField.datatype != 7)
  {
    RCLCPP_WARN_STREAM(logger_,"X, Y, Z fields do not contain floats!");
    return false;
  }

  if (!userFound && field_ != "depth") {
    RCLCPP_WARN_STREAM(logger_, "Requested field '" << field_ << "' not found in point cloud!");
    return false;
  }

  return true;
}

sensor_msgs::msg::PointCloud2 PointCloud2Subscriber::TransformFrame(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &input_msg, std::string frame_id)
{
  RCLCPP_DEBUG_STREAM(logger_,"  Transform");
  sensor_msgs::msg::PointCloud2 output_cloud;  
  geometry_msgs::msg::TransformStamped transform;
  try
  {
    transform_optical_.header.stamp = input_msg->header.stamp;
    transform_optical_.header.frame_id = input_msg->header.frame_id;
    if(frame_id_ == "") {
      transform = transform_optical_;
    } else {
      if(tf_buffer_->canTransform(frame_id, input_msg->header.frame_id, input_msg->header.stamp, rclcpp::Duration::from_seconds(wait_for_tf_delay_)))
      {
        transform = tf_buffer_->lookupTransform(frame_id, input_msg->header.frame_id, input_msg->header.stamp);
        
      // Transform into a z-forward orientation of requested frame for opencv
      tf2::doTransform(transform.transform, transform.transform, transform_optical_);
      }
      else
      {
        RCLCPP_WARN_STREAM(logger_, "  PointCloud2 subscriber is waiting for transform from " << input_msg->header.frame_id << " to " << frame_id << " to become available.");
        transform = transform_optical_;
      }
    }
    tf2::doTransform(*input_msg, output_cloud, transform);
  }
  catch (tf2::TransformException &ex) 
  {
    RCLCPP_WARN_STREAM(logger_, "  PointCloud2 subscriber: " << ex.what());
    output_cloud = *input_msg;        
  }

  return output_cloud;
}

void PointCloud2Subscriber::GatherCameraInfo(cv::Mat &intrinsic_matrix, cv::Mat &distortion_coefficients)
{
  // Setup camera_info
  distortion_coefficients = cv::Mat::zeros(1, 5, CV_64F);

  // Intrinsic Matrix
  intrinsic_matrix = cv::Mat::zeros(3, 3, CV_64F);
  intrinsic_matrix.at<double>(0,0) = focal_length_;
  intrinsic_matrix.at<double>(1,1) = focal_length_;
  intrinsic_matrix.at<double>(0,2) = width_/2.0;
  intrinsic_matrix.at<double>(1,2) = height_/2.0;
  intrinsic_matrix.at<double>(2,2) = 1.0;
}

bool PointCloud2Subscriber::CreateUserImage(const std_msgs::msg::Header &cloud_header, const sensor_msgs::msg::PointField &userField, cv_bridge::CvImage& userImage)
{ 
  userImage.header = cloud_header;
  if(frame_id_ != "") userImage.header.frame_id = frame_id_;
  if((userField.datatype == sensor_msgs::msg::PointField::UINT8   && userField.count == 4) ||
      (userField.datatype == sensor_msgs::msg::PointField::UINT32  && (userField.name == "rgb" || userField.name == "rgba")) ||
      (userField.datatype == sensor_msgs::msg::PointField::FLOAT32 && (userField.name == "rgb" || userField.name == "rgba"))
    )
  {
     RCLCPP_DEBUG_STREAM(logger_,"      4 8-bit unsigned integers");       
    userImage.encoding = sensor_msgs::image_encodings::TYPE_8UC3;
    userImage.image = cv::Mat::zeros(height_, width_, CV_8UC3);
  }
  else if((userField.datatype == sensor_msgs::msg::PointField::UINT16  && userField.count == 4) ||
          (userField.datatype == sensor_msgs::msg::PointField::FLOAT64 && (userField.name == "rgb" || userField.name == "rgba"))
          )
  {
     RCLCPP_DEBUG_STREAM(logger_,"      4 16-bit unsigned integers");    
    userImage.encoding = sensor_msgs::image_encodings::TYPE_16UC3;
    userImage.image = cv::Mat::zeros(height_, width_, CV_16UC3);
  }
  else if((userField.datatype == sensor_msgs::msg::PointField::UINT8))
  {
     RCLCPP_DEBUG_STREAM(logger_,"      1 8-bit unsigned integer");    
    userImage.encoding = sensor_msgs::image_encodings::TYPE_8UC1;
    userImage.image = cv::Mat::zeros(height_, width_, CV_8UC1);         
  }
  else if((userField.datatype == sensor_msgs::msg::PointField::INT8))
  {
     RCLCPP_DEBUG_STREAM(logger_,"      1 8-bit signed integer");             
    userImage.encoding = sensor_msgs::image_encodings::TYPE_8SC1;
    userImage.image = cv::Mat::zeros(height_, width_, CV_8SC1);            
  }        
  else if((userField.datatype == sensor_msgs::msg::PointField::UINT16))
  {
     RCLCPP_DEBUG_STREAM(logger_,"      1 16-bit unsigned integer");               
    userImage.encoding = sensor_msgs::image_encodings::TYPE_16UC1;
    userImage.image = cv::Mat::zeros(height_, width_, CV_16UC1);
  }
  else if((userField.datatype == sensor_msgs::msg::PointField::INT16))
  {
     RCLCPP_DEBUG_STREAM(logger_,"      1 16-bit signed integer");               
    userImage.encoding = sensor_msgs::image_encodings::TYPE_16SC1;
    userImage.image = cv::Mat::zeros(height_, width_, CV_16SC1);  
  }
  else if((userField.datatype == sensor_msgs::msg::PointField::FLOAT32))
  {
     RCLCPP_DEBUG_STREAM(logger_,"      1 32-bit float");     
    userImage.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
    userImage.image = cv::Mat::zeros(height_, width_, CV_32FC1);            
  }        
  else if((userField.datatype == sensor_msgs::msg::PointField::FLOAT64))
  {
     RCLCPP_DEBUG_STREAM(logger_,"      1 64-bit float");               
    userImage.encoding = sensor_msgs::image_encodings::TYPE_64FC1;
    userImage.image = cv::Mat::zeros(height_, width_, CV_64FC1);  
  }        
  else if(field_ != "depth")
  {
    RCLCPP_WARN_STREAM(logger_, "Requested field '" << field_ << "' not found in point cloud!");
    return false;
  }
  return true;
}

bool PointCloud2Subscriber::CreateDepthImage(const std_msgs::msg::Header &cloud_header, cv_bridge::CvImage& depthImage)
{ 
  depthImage.header = cloud_header;
  if(frame_id_ != "") depthImage.header.frame_id = frame_id_;
  depthImage.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
  depthImage.image = cv::Mat::ones(height_, width_, CV_32FC1) * std::numeric_limits<float>::max();

  return true;
}

std::vector<cv::Point2f> PointCloud2Subscriber::ProjectPoints(const sensor_msgs::msg::PointCloud2 &output_cloud,
      const sensor_msgs::msg::PointField &xField, const sensor_msgs::msg::PointField &yField, const sensor_msgs::msg::PointField &zField,
      const cv::Mat &intrinsic_matrix, const cv::Mat &distortion_coefficients, std::vector<cv::Point3f> &obj_pts)
{
  // Create 3D points for projection
  const size_t size = output_cloud.height * output_cloud.width;
  RCLCPP_DEBUG_STREAM(logger_,"  Create 3D points for projection: " << size);
  obj_pts.reserve(size); // Reserve space for better performance

  for (size_t i = 0; i < size; ++i)
  {
    // Find index of xyz data
    size_t point_start = i * output_cloud.point_step;
    size_t x_access = point_start + xField.offset;
    size_t y_access = point_start + yField.offset;
    size_t z_access = point_start + zField.offset;
    // Extract X, Y, Z coordinates
    float X,Y,Z;
    std::memcpy(&X, &output_cloud.data[x_access], sizeof(float));
    std::memcpy(&Y, &output_cloud.data[y_access], sizeof(float));
    std::memcpy(&Z, &output_cloud.data[z_access], sizeof(float));
    obj_pts.push_back(cv::Point3f(X, Y, Z));
  }

  std::vector<cv::Point2f> img_pts;
  if (!obj_pts.empty())  // cv::projectPoints asserts (throws) on empty input
  {
    cv::Mat rvec = cv::Mat::zeros(3, 1, cv::DataType<double>::type);
    cv::Mat tvec = cv::Mat::zeros(3, 1, cv::DataType<double>::type);
    cv::projectPoints(obj_pts, rvec, tvec, intrinsic_matrix, distortion_coefficients, img_pts);
  }
  return img_pts;
}

cv_bridge::CvImage PointCloud2Subscriber::NormalizeImage(const cv_bridge::CvImage &inputImage)
{
  cv_bridge::CvImage normalized_image;
  if(inputImage.encoding == sensor_msgs::image_encodings::TYPE_16UC3) {
    cv::normalize(inputImage.image, normalized_image.image, 0, 65535, cv::NORM_MINMAX);
    normalized_image.header = inputImage.header;
    normalized_image.encoding = inputImage.encoding;
  } else if(inputImage.encoding == sensor_msgs::image_encodings::TYPE_8UC3) {
    cv::normalize(inputImage.image, normalized_image.image, 0, 255, cv::NORM_MINMAX);
    normalized_image.header = inputImage.header;
    normalized_image.encoding = inputImage.encoding;    
  } else if(inputImage.encoding == sensor_msgs::image_encodings::TYPE_16UC1) {
    cv::normalize(inputImage.image, normalized_image.image, 0, 65535, cv::NORM_MINMAX);
    normalized_image.header = inputImage.header;
    normalized_image.encoding = inputImage.encoding;    
  } else if(inputImage.encoding == sensor_msgs::image_encodings::TYPE_16SC1) {
    cv::normalize(inputImage.image, normalized_image.image, -32768, 32767, cv::NORM_MINMAX);
    normalized_image.header = inputImage.header;
    normalized_image.encoding = inputImage.encoding;    
  } else if(inputImage.encoding == sensor_msgs::image_encodings::TYPE_8UC1) {
    cv::normalize(inputImage.image, normalized_image.image, 0, 255, cv::NORM_MINMAX);
    normalized_image.header = inputImage.header;
    normalized_image.encoding = inputImage.encoding;        
  } else if(inputImage.encoding == sensor_msgs::image_encodings::TYPE_8SC1) {
    cv::normalize(inputImage.image, normalized_image.image, -128, 127, cv::NORM_MINMAX);
    normalized_image.header = inputImage.header;
    normalized_image.encoding = inputImage.encoding;            
  } else {
   normalized_image = inputImage;
  }

  return normalized_image;
}
cv_bridge::CvImage PointCloud2Subscriber::ConvertToColor(const cv::Mat &depthMask, const cv_bridge::CvImage &inputImage)
{
  // Create gradient background image (BGR format for depth visualization)
  cv::Mat gradientBackground = cv::Mat::zeros(height_, width_, CV_8UC3);
  for (int row = 0; row < height_; row++) {
    for (int col = 0; col < width_; col++) {
      // Create vertical saturation-based gradient from saturated medium blue at top to black at bottom
      // Medium blue color: RGB(0, 80, 200) -> BGR(200, 80, 0)
      float saturation_ratio = 1.0f - (float)row / (float)height_; // 1.0 at top (full saturation), 0.0 at bottom (black)
      uint8_t blue_value = (uint8_t)(200 * saturation_ratio);   // Blue channel (200->0)
      uint8_t green_value = (uint8_t)(80 * saturation_ratio);   // Green channel (80->0)
      uint8_t red_value = (uint8_t)(0 * saturation_ratio);      // Red channel (0->0)
      
      cv::Vec3b& pixel = gradientBackground.at<cv::Vec3b>(row, col);
      pixel[0] = blue_value;  // B
      pixel[1] = green_value; // G
      pixel[2] = red_value;   // R
    }
  }
  
  // Composite the final image by blending data with gradient background
  // Where mask == 255 (has data): use actual data values
  // Where mask == 0 (no data): use gradient background values
  RCLCPP_DEBUG_STREAM(logger_, "  Final Compositing");
  // Setup color image
   RCLCPP_DEBUG_STREAM(logger_,"    Color");        
  cv_bridge::CvImage colorImage;
  colorImage.header = inputImage.header;
  if(frame_id_ != "") colorImage.header.frame_id = frame_id_;  
  if(inputImage.encoding == sensor_msgs::image_encodings::TYPE_32FC1) {
    RCLCPP_DEBUG_STREAM(logger_,"Converting TYPE_32FC1 to color");      
    colorImage.encoding = sensor_msgs::image_encodings::BGR8;
    colorImage.image = cv::Mat::zeros(height_, width_, CV_8UC3);
    for (int row = 0; row < height_; row++) {
      for (int col = 0; col < width_; col++) {
        if (depthMask.at<uint8_t>(row, col) == 255) {
          // Pixel has real depth data - convert depth to grayscale and display as white/gray
          float depth_value = inputImage.image.at<float>(row, col);
          // Normalize depth to 0-255 range (assuming max depth ~100 meters)
          uint8_t intensity = static_cast<uint8_t>(std::min(255.0f, depth_value * 2.55f)); // 100m -> 255
          
          cv::Vec3b& pixel = colorImage.image.at<cv::Vec3b>(row, col);
          pixel[0] = intensity; // B
          pixel[1] = intensity; // G  
          pixel[2] = intensity; // R (grayscale)
        } else {
          // Pixel has no depth data - use gradient background value
          colorImage.image.at<cv::Vec3b>(row, col) = gradientBackground.at<cv::Vec3b>(row, col);
        }
      }
    }
  } else if(inputImage.encoding == sensor_msgs::image_encodings::TYPE_32FC3) {
    RCLCPP_DEBUG_STREAM(logger_,"Converting TYPE_32FC3 to color");    
    colorImage.encoding = sensor_msgs::image_encodings::BGR8;
    colorImage.image = cv::Mat::zeros(height_, width_, CV_8UC3);
    for (int row = 0; row < height_; row++) {
      for (int col = 0; col < width_; col++) {
        if (depthMask.at<uint8_t>(row, col) == 255) {
          // Pixel has real depth data - convert depth to grayscale and display as white/gray
          cv::Vec3f depth_value = inputImage.image.at<cv::Vec3f>(row, col);
          
          cv::Vec3b& pixel = colorImage.image.at<cv::Vec3b>(row, col);
          pixel[0] = static_cast<uint8_t>(std::min(255.0f, depth_value[0] * 2.55f)); // B
          pixel[1] = static_cast<uint8_t>(std::min(255.0f, depth_value[1] * 2.55f)); // G  
          pixel[2] = static_cast<uint8_t>(std::min(255.0f, depth_value[2] * 2.55f)); // R (grayscale)
        } else {
          // Pixel has no depth data - use gradient background value
          colorImage.image.at<cv::Vec3b>(row, col) = gradientBackground.at<cv::Vec3b>(row, col);
        }
      }
    }
  } else if(inputImage.encoding == sensor_msgs::image_encodings::TYPE_16UC3) {
    RCLCPP_DEBUG_STREAM(logger_,"Converting TYPE_16UC3 to color");
    cv::Mat gradientBackground16;
    gradientBackground.convertTo(gradientBackground16, CV_16UC3);
    colorImage.encoding = sensor_msgs::image_encodings::BGR16;
    colorImage.image = cv::Mat::zeros(height_, width_, CV_16UC3);
    for (int row = 0; row < height_; row++) {
      for (int col = 0; col < width_; col++) {
        if (depthMask.at<uint8_t>(row, col) == 255) {
          colorImage.image.at<cv::Vec3w>(row, col) = inputImage.image.at<cv::Vec3w>(row, col);
        }
        else
        {
          // Pixel has no depth data - use gradient background value scaled to 16 bit
          colorImage.image.at<cv::Vec3w>(row, col) = gradientBackground16.at<cv::Vec3w>(row, col);
        }
      }
    }
  } else if(inputImage.encoding == sensor_msgs::image_encodings::TYPE_8UC3) {
    RCLCPP_DEBUG_STREAM(logger_,"Converting TYPE_8UC3 to color");
    colorImage.encoding = sensor_msgs::image_encodings::BGR8;
    colorImage.image = cv::Mat::zeros(height_, width_, CV_8UC3);
    for (int row = 0; row < height_; row++) {
      for (int col = 0; col < width_; col++) {
        if (depthMask.at<uint8_t>(row, col) == 255) {
          colorImage.image.at<cv::Vec3b>(row, col) = inputImage.image.at<cv::Vec3b>(row, col);
        } else {
          // Pixel has no depth data - use gradient background value
          colorImage.image.at<cv::Vec3b>(row, col) = gradientBackground.at<cv::Vec3b>(row, col);
        }
      }
    }    
  } else if(inputImage.encoding == sensor_msgs::image_encodings::TYPE_8UC1) {
    RCLCPP_DEBUG_STREAM(logger_,"Converting TYPE_8UC1 to color");
    colorImage.encoding = sensor_msgs::image_encodings::BGR8;
    colorImage.image = cv::Mat::zeros(height_, width_, CV_8UC3);      
    for (int row = 0; row < height_; row++) {
      for (int col = 0; col < width_; col++) {
        if (depthMask.at<uint8_t>(row, col) == 255) {
          // Pixel has real depth data - convert depth to grayscale and display as white/gray
          float depth_value = inputImage.image.at<uint8_t>(row, col);
          // Pass 8-bit grayscale value directly as intensity
          uint8_t intensity = depth_value;
          
          cv::Vec3b& pixel = colorImage.image.at<cv::Vec3b>(row, col);
          pixel[0] = intensity; // B
          pixel[1] = intensity; // G  
          pixel[2] = intensity; // R (grayscale)
        } else {
          // Pixel has no depth data - use gradient background value
          colorImage.image.at<cv::Vec3b>(row, col) = gradientBackground.at<cv::Vec3b>(row, col);
        }
      }
    }
  } else if(inputImage.encoding == sensor_msgs::image_encodings::TYPE_8SC1) {
    RCLCPP_DEBUG_STREAM(logger_,"Converting TYPE_8SC1 to color");
    colorImage.encoding = sensor_msgs::image_encodings::BGR8;
    colorImage.image = cv::Mat::zeros(height_, width_, CV_8UC3);      
    for (int row = 0; row < height_; row++) {
      for (int col = 0; col < width_; col++) {
        if (depthMask.at<uint8_t>(row, col) == 255) {
          // Pixel has real depth data - convert depth to grayscale and display as white/gray
          int8_t depth_value = inputImage.image.at<int8_t>(row, col);
          // Normalize depth to 0-255 range
          uint8_t intensity = static_cast<uint8_t>(depth_value + 128);
          
          cv::Vec3b& pixel = colorImage.image.at<cv::Vec3b>(row, col);
          pixel[0] = intensity; // B
          pixel[1] = intensity; // G  
          pixel[2] = intensity; // R (grayscale)
        } else {
          // Pixel has no depth data - use gradient background value
          colorImage.image.at<cv::Vec3b>(row, col) = gradientBackground.at<cv::Vec3b>(row, col);
        }
      }
    }
  } else if(inputImage.encoding == sensor_msgs::image_encodings::TYPE_16UC1) {
    RCLCPP_DEBUG_STREAM(logger_,"Converting TYPE_16UC1 to color");
    cv::Mat gradientBackground16;
    gradientBackground.convertTo(gradientBackground16, CV_16UC3, 257.0, 0.0);    
    colorImage.encoding = sensor_msgs::image_encodings::BGR16;
    colorImage.image = cv::Mat::zeros(height_, width_, CV_16UC3); 
    for (int row = 0; row < height_; row++) {
      for (int col = 0; col < width_; col++) {
        if (depthMask.at<uint8_t>(row, col) == 255) {
          // Pixel has real depth data - convert depth to grayscale and display as white/gray
          uint16_t depth_value = inputImage.image.at<uint16_t>(row, col);
          // Pass 16-bit depth value directly as grayscale intensity
          uint16_t intensity = static_cast<uint16_t>(depth_value);
          
          cv::Vec3w& pixel = colorImage.image.at<cv::Vec3w>(row, col);
          pixel[0] = intensity; // B
          pixel[1] = intensity; // G  
          pixel[2] = intensity; // R (grayscale)
        } else {
          // Pixel has no depth data - use gradient background value scaled to 16 bit
          colorImage.image.at<cv::Vec3w>(row, col) = gradientBackground16.at<cv::Vec3w>(row, col);
        }
      }
    }
  } else if(inputImage.encoding == sensor_msgs::image_encodings::TYPE_16SC1) {
    RCLCPP_DEBUG_STREAM(logger_,"Converting TYPE_16SC1 to color");
    cv::Mat gradientBackground16;
    gradientBackground.convertTo(gradientBackground16, CV_16UC3, 257.0, 0.0);
    colorImage.encoding = sensor_msgs::image_encodings::BGR16;
    colorImage.image = cv::Mat::zeros(height_, width_, CV_16UC3);
    for (int row = 0; row < height_; row++) {
      for (int col = 0; col < width_; col++) {
        if (depthMask.at<uint8_t>(row, col) == 255) {
          // Pixel has real depth data - convert depth to grayscale and display as white/gray
          int16_t depth_value = inputImage.image.at<int16_t>(row, col);
          // Shift signed range [-32768,32767] to unsigned [0,65535]
          uint16_t intensity = static_cast<uint16_t>(depth_value + 32768);
          
          cv::Vec3w& pixel = colorImage.image.at<cv::Vec3w>(row, col);
          pixel[0] = intensity; // B
          pixel[1] = intensity; // G  
          pixel[2] = intensity; // R (grayscale)
        } else {
          // Pixel has no depth data - use gradient background value scaled to 16 bit
          colorImage.image.at<cv::Vec3w>(row, col) = gradientBackground16.at<cv::Vec3w>(row, col);
        }
      }
    }    
  } else {
    RCLCPP_DEBUG_STREAM(logger_,"Cannot convert to color: " << inputImage.encoding.c_str());    
    colorImage = inputImage;
  }
  
  return colorImage;
}

std::shared_ptr<SubscriberInterface> PointCloud2SubscriberFactory::create_subscriber(
    rclcpp::Node::SharedPtr node)
{
  return std::make_shared<PointCloud2Subscriber>(node);
}

std::vector<std::string> PointCloud2SubscriberFactory::get_available_topics(
  rclcpp::Node & node
) {
  std::vector<std::string> result;
  auto topic_names_and_types = node.get_topic_names_and_types();
  for (const auto & topic_and_types : topic_names_and_types) {
    for (const auto & type : topic_and_types.second) {
      if (type == this->get_type()) {
        result.push_back(topic_and_types.first);
        break;
      }
    }
  }
  return result;
}

}  // namespace subscribers
}  // namespace web_video_server

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  web_video_server::subscribers::PointCloud2SubscriberFactory,
  web_video_server::SubscriberFactoryInterface)