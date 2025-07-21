#include <math.h>
#include <cmath>
#include "web_video_server/subscribers/pointcloud2_subscriber.hpp"
#include <algorithm>
#include <std_msgs/msg/header.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tf2/convert.h>

#ifdef CV_BRIDGE_USES_OLD_HEADERS
#include <cv_bridge/cv_bridge.h>
#else
#include <cv_bridge/cv_bridge.hpp>
#endif

namespace web_video_server
{

PointCloud2Subscriber::PointCloud2Subscriber(rclcpp::Node::SharedPtr node)
: RosSubscriber(node)
{
  // Initialize our TF items
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  
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
}

void PointCloud2Subscriber::subscribe(const async_web_server_cpp::HttpRequest &request,
                                         const std::string& topic, 
                                         const ImageCallback& callback)
{
  callback_ = callback;
  qos_profile_name_ = request.get_query_param_value_or_default("qos_profile", "default");

  frame_id_ = request.get_query_param_value_or_default("frame_id", "base_link");
  wait_for_tf_delay_ = 0.10;
  field_ = request.get_query_param_value_or_default("field", "depth");
  height_ = request.get_query_param_value_or_default<int>("height", 600);
  width_  = request.get_query_param_value_or_default<int>("width", 800);
  pixel_size_ = request.get_query_param_value_or_default<int>("pixel_size", 5);
  focal_length_ = request.get_query_param_value_or_default<double>("focal_length", 300.0);

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
  
  ros_sub_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(topic, qos, std::bind(&PointCloud2Subscriber::subscriberCallback, this, std::placeholders::_1));
}

void PointCloud2Subscriber::subscriberCallback(const sensor_msgs::msg::PointCloud2::ConstPtr &input_msg)
{
  RCLCPP_DEBUG_STREAM(node_->get_logger(),"Update PointCloud2: " << frame_id_);

  sensor_msgs::msg::PointCloud2 output_cloud;
        
  if (input_msg->data.size() == 0)
  {
    RCLCPP_WARN_STREAM_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000, "No data in pointcloud!");
    output_cloud = *input_msg;
    return;
  }

  auto beginTime = std::chrono::steady_clock::now();

  //transform
   RCLCPP_DEBUG_STREAM(node_->get_logger(),"  Transform");
  geometry_msgs::msg::TransformStamped transform;
  try
  {
    transform = tf_buffer_->lookupTransform(frame_id_, input_msg->header.frame_id, input_msg->header.stamp);
      
      // Create composed transform: first apply spatial transform, then optical frame orientation
      geometry_msgs::msg::TransformStamped composed_transform;
      composed_transform.header = transform.header;
      composed_transform.header.frame_id = frame_id_;
      composed_transform.child_frame_id = input_msg->header.frame_id + "_optical";
      
      // Compose transforms: T_composed = T_optical * T_spatial
      // This applies the spatial transform first, then the optical frame rotation
      tf2::Transform tf_spatial, tf_optical, tf_composed;
      tf2::fromMsg(transform.transform, tf_spatial);
      tf2::fromMsg(transform_optical_.transform, tf_optical);
      // tf_composed = tf_optical * tf_spatial;
      tf_composed = tf_spatial * tf_optical;
      composed_transform.transform = tf2::toMsg(tf_composed);
      
      tf2::doTransform(*input_msg, output_cloud, composed_transform);
  }
  catch (tf2::TransformException &ex) 
  {
    RCLCPP_WARN_STREAM(node_->get_logger(),"  Publish Thread: " << ex.what());
    output_cloud = *input_msg;
    return;
  }

  //Sort input cloud fields by field offset - IMPORTANT: Use input_msg fields, not output_cloud
   RCLCPP_DEBUG_STREAM(node_->get_logger(),"  Sort Fields");
  std::vector<sensor_msgs::msg::PointField> sortedFields(input_msg->fields);
  std::sort(sortedFields.begin(), sortedFields.end(), PointCloud2Subscriber::compareFieldsOffset);

  // Find fields we need in the cloud
  bool xFound = false, yFound = false, zFound = false;
  bool userFound = false;
  sensor_msgs::msg::PointField xField, yField, zField, userField;
  for (size_t i = 0; i < sortedFields.size(); i++)
  {
    sensor_msgs::msg::PointField currentField = sortedFields[i];
    
    if(currentField.name == "x")
    {
      xFound = true;
      xField = currentField;
      RCLCPP_DEBUG_STREAM(node_->get_logger(),"   Found X Field (datatype: " << currentField.datatype << ")");
    }
    else if(currentField.name == "y")
    {
      yFound = true;
      yField = currentField;
      RCLCPP_DEBUG_STREAM(node_->get_logger(),"   Found Y Field (datatype: " << currentField.datatype << ")");          
    }
    else if(currentField.name == "z")
    {
      zFound = true;
      zField = currentField;
      RCLCPP_DEBUG_STREAM(node_->get_logger(),"   Found Z Field (datatype: " << currentField.datatype << ")");          
    }
    if(currentField.name == field_)
    {
      userFound = true;
      userField = currentField;
      RCLCPP_DEBUG_STREAM(node_->get_logger(),"   Found " << currentField.name << " (datatype: " << currentField.datatype << ")");         
    }
  }

  //Check found fields
  if(!xFound || !yFound || !zFound)
  {
    RCLCPP_WARN_STREAM(node_->get_logger(),"Cloud does not contain XYZ data!");
    return;
  }

  // Setup camera_info
   RCLCPP_DEBUG_STREAM(
    node_->get_logger(),"    Camera Info");        
  sensor_msgs::msg::CameraInfo info;
  info.header = output_cloud.header;
  info.header.frame_id = frame_id_;        
  info.height = height_;
  info.width = width_;
  info.distortion_model =  "plumb_bob";
  cv::Mat distortionCoefficients = cv::Mat::zeros(1, 5, CV_64F);
  info.d = distortionCoefficients;

  // Intrinsic Matrix
  cv::Mat intrinsicMatrix = cv::Mat::zeros(3, 3, CV_64F);
  intrinsicMatrix.at<double>(0,0) = focal_length_;
  intrinsicMatrix.at<double>(1,1) = focal_length_;
  intrinsicMatrix.at<double>(0,2) = width_/2;
  intrinsicMatrix.at<double>(1,2) = height_/2;
  intrinsicMatrix.at<double>(2,2) = 1.0;
  for (int row = 0; row < 3; row++)
  {
    for (int col = 0; col < 3; col++)
    {
      info.k[row * 3 + col] = intrinsicMatrix.at<double>(row, col);
    }
  }

  // Rectification Matrix
  cv::Mat rectificationMatrix = cv::Mat::zeros(3, 3, CV_64F);
  rectificationMatrix.at<double>(0,0) = 1.0;
  rectificationMatrix.at<double>(1,1) = 1.0;
  rectificationMatrix.at<double>(2,2) = 1.0;        
  for (int row = 0; row < 3; row++)
  {
    for (int col = 0; col < 3; col++)
    {
      info.r[row * 3 + col] = rectificationMatrix.at<double>(row, col);
    }
  }        

  // Projection Matrix
  cv::Mat projectionMatrix = cv::Mat::zeros(3, 4, CV_64F);        
  projectionMatrix.at<double>(0,0) = focal_length_;
  projectionMatrix.at<double>(1,1) = focal_length_;
  projectionMatrix.at<double>(0,2) = width_/2;
  projectionMatrix.at<double>(1,2) = height_/2;
  projectionMatrix.at<double>(2,2) = 1.0;        
  for (int row = 0; row < 3; row++)
  {
    for (int col = 0; col < 4; col++)
    {
      info.p[row * 4 + col] = projectionMatrix.at<double>(row, col);
    }
  }

  // Setup color image
  //  RCLCPP_DEBUG_STREAM(node_->get_logger(),"    Color");        
  // cv_bridge::CvImage colorImage;
  // colorImage.header = output_cloud.header;
  // colorImage.header.frame_id = frame_id_;
  // colorImage.encoding = sensor_msgs::image_encodings::BGR8;
  // colorImage.image = cv::Mat::zeros(height_, width_, 'bgr8');

   // Setup depth rendering with mask-based approach
  RCLCPP_DEBUG_STREAM(node_->get_logger(),"    Depth");           
  
  // MASK-BASED DEPTH RENDERING APPROACH:
  // 1. Create a pure depth buffer (no background) for z-buffering
  // 2. Create a mask to track which pixels have real depth data
  // 3. Render depth data using proper z-buffering
  // 4. Composite final image by blending depth buffer with gradient background
  
  // Step 1: Create depth buffer initialized to infinity (no background values)
  cv::Mat depthBuffer = cv::Mat::ones(height_, width_, CV_32FC1) * std::numeric_limits<float>::max();
  
  // Step 2: Create mask to track pixels with real depth data (false = no data, true = has data)
  cv::Mat depthMask = cv::Mat::zeros(height_, width_, CV_8UC1);
  
  // Step 3: Create gradient background image (BGR format for depth visualization)
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
  
  // Step 4: Create final depth image as BGR for color visualization      
  cv_bridge::CvImage depthImage;
  depthImage.header = output_cloud.header;
  depthImage.header.frame_id = frame_id_;
  // depthImage.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
  // depthImage.image = cv::Mat::zeros(height_, width_, CV_32FC1);
  depthImage.encoding = sensor_msgs::image_encodings::BGR8;
  depthImage.image = cv::Mat::zeros(height_, width_, CV_8UC3);

  // Setup user image
   RCLCPP_DEBUG_STREAM(node_->get_logger(),"    User");         
   RCLCPP_DEBUG_STREAM(node_->get_logger(),"    User-Datatype: " << +userField.datatype);  
  cv_bridge::CvImage userImage;
  userImage.header = output_cloud.header;
  userImage.header.frame_id = frame_id_;
  if((userField.datatype == sensor_msgs::msg::PointField::UINT8   && userField.count == 4) ||
      (userField.datatype == sensor_msgs::msg::PointField::UINT32  && (userField.name == "rgb" || userField.name == "rgba")) ||
      (userField.datatype == sensor_msgs::msg::PointField::FLOAT32 && (userField.name == "rgb" || userField.name == "rgba"))
    )
  {
     RCLCPP_DEBUG_STREAM(node_->get_logger(),"      4 8-bit unsigned integers");       
    userImage.encoding = sensor_msgs::image_encodings::TYPE_8UC3;
    userImage.image = cv::Mat::zeros(height_, width_, CV_8UC3);
    // Create dark blue saturation-based gradient background for 3-channel image
    for (int row = 0; row < height_; row++) {
      for (int col = 0; col < width_; col++) {
        // BGR format: create vertical saturation-based gradient from medium blue at top to black at bottom
        // Medium blue color: RGB(0, 80, 200) -> BGR(200, 80, 0)
        float saturation_ratio = 1.0f - (float)row / (float)height_; // 1.0 at top (full saturation), 0.0 at bottom (black)
        uint8_t blue_value = (uint8_t)(200 * saturation_ratio);   // Blue channel (200->0)
        uint8_t green_value = (uint8_t)(80 * saturation_ratio);   // Green channel (80->0)
        uint8_t red_value = (uint8_t)(0 * saturation_ratio);      // Red channel (0->0)
        cv::Vec3b& pixel = userImage.image.at<cv::Vec3b>(row, col);
        pixel[0] = blue_value;  // B
        pixel[1] = green_value; // G
        pixel[2] = red_value;   // R
      }
    }
  }
  else if((userField.datatype == sensor_msgs::msg::PointField::UINT16  && userField.count == 4) ||
          (userField.datatype == sensor_msgs::msg::PointField::FLOAT64 && (userField.name == "rgb" || userField.name == "rgba"))
          )
  {
     RCLCPP_DEBUG_STREAM(node_->get_logger(),"      4 16-bit unsigned integers");    
    userImage.encoding = sensor_msgs::image_encodings::TYPE_16UC3;
    userImage.image = cv::Mat::zeros(height_, width_, CV_16UC3);
    // Create dark blue saturation-based gradient background for 16-bit 3-channel image
    for (int row = 0; row < height_; row++) {
      for (int col = 0; col < width_; col++) {
        // BGR format: create vertical saturation-based gradient from medium blue at top to black at bottom
        // Medium blue color scaled to 16-bit: RGB(0, 80, 200) -> BGR(51400, 20560, 0)
        float saturation_ratio = 1.0f - (float)row / (float)height_; // 1.0 at top (full saturation), 0.0 at bottom (black)
        uint16_t blue_value = (uint16_t)(51400 * saturation_ratio);   // Blue channel (51400->0)
        uint16_t green_value = (uint16_t)(20560 * saturation_ratio);  // Green channel (20560->0)
        uint16_t red_value = (uint16_t)(0 * saturation_ratio);        // Red channel (0->0)
        cv::Vec3w& pixel = userImage.image.at<cv::Vec3w>(row, col);
        pixel[0] = blue_value;  // B
        pixel[1] = green_value; // G
        pixel[2] = red_value;   // R
      }
    }
  }
  else if((userField.datatype == sensor_msgs::msg::PointField::UINT8))
  {
     RCLCPP_DEBUG_STREAM(node_->get_logger(),"      1 8-bit unsigned integer");    
    userImage.encoding = sensor_msgs::image_encodings::TYPE_8UC1;
    userImage.image = cv::Mat::zeros(height_, width_, CV_8UC1); 
    // Initialize with zeros - gradient background will be applied during compositing     
  }
  else if((userField.datatype == sensor_msgs::msg::PointField::INT8))
  {
     RCLCPP_DEBUG_STREAM(node_->get_logger(),"      1 8-bit signed integer");             
    userImage.encoding = sensor_msgs::image_encodings::TYPE_8SC1;
    userImage.image = cv::Mat::zeros(height_, width_, CV_8SC1); 
    // Initialize with zeros - gradient background will be applied during compositing           
  }        
  else if((userField.datatype == sensor_msgs::msg::PointField::UINT16))
  {
     RCLCPP_DEBUG_STREAM(node_->get_logger(),"      1 16-bit unsigned integer");               
    userImage.encoding = sensor_msgs::image_encodings::TYPE_16UC1;
    userImage.image = cv::Mat::zeros(height_, width_, CV_16UC1);    
    // Initialize with zeros - gradient background will be applied during compositing        
  }
  else if((userField.datatype == sensor_msgs::msg::PointField::INT16))
  {
     RCLCPP_DEBUG_STREAM(node_->get_logger(),"      1 16-bit signed integer");               
    userImage.encoding = sensor_msgs::image_encodings::TYPE_16SC1;
    userImage.image = cv::Mat::zeros(height_, width_, CV_16SC1);  
    // Initialize with zeros - gradient background will be applied during compositing
  }
  else if((userField.datatype == sensor_msgs::msg::PointField::FLOAT32))
  {
     RCLCPP_DEBUG_STREAM(node_->get_logger(),"      1 32-bit float");     
    userImage.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
    userImage.image = cv::Mat::zeros(height_, width_, CV_32FC1);   
    // Initialize with zeros - gradient background will be applied during compositing         
  }        
  else if((userField.datatype == sensor_msgs::msg::PointField::FLOAT64))
  {
     RCLCPP_DEBUG_STREAM(node_->get_logger(),"      1 64-bit float");               
    userImage.encoding = sensor_msgs::image_encodings::TYPE_64FC1;
    userImage.image = cv::Mat::zeros(height_, width_, CV_64FC1);  
    // Initialize with zeros - gradient background will be applied during compositing
  }        
  else if(field_ != "depth" && !userFound)
  {
    RCLCPP_WARN_STREAM(node_->get_logger(), "Requested field '" << field_ << "' not found in point cloud!");
    return;
  }

  // Setup OpenCV matrixes
   RCLCPP_DEBUG_STREAM(node_->get_logger(),"  Setup opencv matrixes");        
  cv::Mat rvec = cv::Mat::zeros(3, 1, cv::DataType<double>::type);
  cv::Mat tvec = cv::Mat::zeros(3, 1, cv::DataType<double>::type);
  std::vector<cv::Point3f> obj_pts;
  std::vector<cv::Point2f> img_pts;

  // #pragma omp parallel for ???????????
  // Create 3D points for projection
  int size = output_cloud.height * output_cloud.width;
  RCLCPP_DEBUG_STREAM(node_->get_logger(),"  Create 3D points for projection: " << size);
  obj_pts.reserve(size); // Reserve space for better performance

  for (int i = 0; i < size; ++i)
  {
    // Check bounds before accessing data buffer
    size_t point_start = i * output_cloud.point_step;
    size_t x_access = point_start + xField.offset;
    size_t y_access = point_start + yField.offset;
    size_t z_access = point_start + zField.offset;
    
    // Validate buffer bounds for coordinate access using correct field sizes
    if (x_access + PointCloud2Subscriber::sizeOfPointField(xField.datatype) > output_cloud.data.size() ||
        y_access + PointCloud2Subscriber::sizeOfPointField(yField.datatype) > output_cloud.data.size() ||
        z_access + PointCloud2Subscriber::sizeOfPointField(zField.datatype) > output_cloud.data.size()) {
      RCLCPP_WARN_STREAM_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000, 
                                   "Buffer access out of bounds for point " << i << ", skipping point");
      continue;
    }
    
    // Extract X, Y, Z coordinates with proper type handling
    float X, Y, Z;

    std::memcpy(&X, &output_cloud.data[i * output_cloud.point_step + xField.offset], sizeof(float));
    std::memcpy(&Y, &output_cloud.data[i * output_cloud.point_step + yField.offset], sizeof(float));
    std::memcpy(&Z, &output_cloud.data[i * output_cloud.point_step + zField.offset], sizeof(float));
    obj_pts.push_back(cv::Point3f(X, Y, Z));
  }
  
  cv::projectPoints(obj_pts, rvec, tvec, intrinsicMatrix, distortionCoefficients, img_pts);

  // Process projected points and render to image
  RCLCPP_DEBUG_STREAM(node_->get_logger(),"  Process projected points: " << img_pts.size());     
  for (size_t i = 0; i < img_pts.size(); ++i)
  {
    int u = int(img_pts[i].x);
    int v = int(img_pts[i].y);
    
    // // Check if point is inside fov of camera
    // if ((u >= 0) && (u < depthImage.image.cols) &&  
    //     (v >= 0) && (v < depthImage.image.rows) &&
    //     obj_pts[i].z > 0)

    // Check if point is inside field of view and has positive depth
    // COMMENTED OUT Z > 0 check: This filters out points behind sensor which may be valid lidar data
    // RELAXED BOUNDS: Allow points slightly outside FOV to capture more lidar data
    if ((u >= -pixel_size_) && (u < width_ + pixel_size_) &&  
        (v >= -pixel_size_) && (v < height_ + pixel_size_))
        // obj_pts[i].z > 0)
    {
      // Buffer bounds checking for user field data access
      // Skip if userField is invalid (only needed for non-depth fields)
      // COMMENTED OUT: This validation might filter out valid lidar data with extended field types
      // if (field_ != "depth" && (userField.datatype == 0 || userField.datatype > 8)) {
      //   continue; // Skip this point if userField is invalid
      // }
      
      size_t point_start = i * output_cloud.point_step;
      size_t user_field_access = point_start + userField.offset;
      size_t user_field_size = (field_ == "depth") ? 4 : PointCloud2Subscriber::sizeOfPointField(userField.datatype); // Use 4 bytes for depth, validate userField for others
      
      // For multi-component fields like RGBA, need to check access for all components
      size_t max_user_field_access = user_field_access;
      if (userField.count == 4) {
        // RGBA field - check access for 4th component
        max_user_field_access = user_field_access + (userField.count - 1) * user_field_size;
      } else if ((userField.datatype == sensor_msgs::msg::PointField::UINT16 && userField.count == 4) ||
                 (userField.datatype == sensor_msgs::msg::PointField::FLOAT64 && (userField.name == "rgb" || userField.name == "rgba"))) {
        // Special cases for 16-bit RGBA or 64-bit RGB
        max_user_field_access = user_field_access + 3 * sizeof(uint16_t); // 3 additional components for RGB
      }
      
      // Validate user field buffer bounds
      if (max_user_field_access + user_field_size > output_cloud.data.size()) {
        RCLCPP_WARN_STREAM_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000, 
                                     "User field buffer access out of bounds for point " << i << ", skipping point");
        continue;
      }
      
      // MASK-BASED DEPTH RENDERING: Step 3 - Process depth points with proper z-buffering
      // Check if this point should be rendered (z-buffer test)
      // Only render if:
      // 1. No depth data exists at this pixel yet (depth buffer == infinity)
      // 2. OR this point is closer than the existing depth value
      float current_depth_buffer = depthBuffer.at<float>(v, u);
      if(current_depth_buffer == std::numeric_limits<float>::max() || current_depth_buffer > obj_pts[i].z)
      {
        // RGB field handling is done in the user field processing below           
        
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
            // RGB color handling is done in the user field processing

            // Update the depth buffer with the closest point
            depthBuffer.at<float>(j, k) = obj_pts[i].z;
            
            // Mark this pixel as having real depth data
            depthMask.at<uint8_t>(j, k) = 255; // 255 = has data, 0 = no data
            
            // user image - write data for the closest point (depth buffer was already updated)
            if( (userField.datatype == sensor_msgs::msg::PointField::UINT8   && userField.count == 4) ||
                (userField.datatype == sensor_msgs::msg::PointField::UINT32  && (userField.name == "rgb" || userField.name == "rgba")) ||
                (userField.datatype == sensor_msgs::msg::PointField::FLOAT32 && (userField.name == "rgb" || userField.name == "rgba"))
              )
            {
              // uint8_t user1, user2, user3;
              // std::memcpy(&user1, &output_cloud.data[i * output_cloud.point_step + userField.offset + 0],sizeof(uint8_t));
              // std::memcpy(&user2, &output_cloud.data[i * output_cloud.point_step + userField.offset + 1],sizeof(uint8_t));
              // std::memcpy(&user3, &output_cloud.data[i * output_cloud.point_step + userField.offset + 2],sizeof(uint8_t));
              // cv::Vec3b pixel;
              // pixel[0] = user1;
              // pixel[1] = user2;
              // pixel[2] = user3;
              // userImage.image.at<cv::Vec3b>(j, k) = pixel;

              // Handle RGB/RGBA fields - these are packed color data
              if (userField.datatype == sensor_msgs::msg::PointField::UINT32) {
                // 32-bit packed RGB (most common format)
                uint32_t rgb_packed;
                std::memcpy(&rgb_packed, &output_cloud.data[i * output_cloud.point_step + userField.offset], sizeof(uint32_t));
                uint8_t r = (rgb_packed >> 16) & 0xFF;
                uint8_t g = (rgb_packed >> 8) & 0xFF;
                uint8_t b = rgb_packed & 0xFF;
                cv::Vec3b pixel;
                pixel[0] = b; // OpenCV uses BGR format
                pixel[1] = g;
                pixel[2] = r;
                userImage.image.at<cv::Vec3b>(j, k) = pixel;
              } else if (userField.datatype == sensor_msgs::msg::PointField::FLOAT32) {
                // 32-bit packed RGB as float
                float rgb_float;
                std::memcpy(&rgb_float, &output_cloud.data[i * output_cloud.point_step + userField.offset], sizeof(float));
                uint32_t rgb_packed = *reinterpret_cast<uint32_t*>(&rgb_float);
                uint8_t r = (rgb_packed >> 16) & 0xFF;
                uint8_t g = (rgb_packed >> 8) & 0xFF;
                uint8_t b = rgb_packed & 0xFF;
                cv::Vec3b pixel;
                pixel[0] = b; // OpenCV uses BGR format
                pixel[1] = g;
                pixel[2] = r;
                userImage.image.at<cv::Vec3b>(j, k) = pixel;
              } else {
                // 4 separate uint8 values (RGBA)
                uint8_t user1, user2, user3;
                std::memcpy(&user1, &output_cloud.data[i * output_cloud.point_step + userField.offset + 0], sizeof(uint8_t));
                std::memcpy(&user2, &output_cloud.data[i * output_cloud.point_step + userField.offset + 1], sizeof(uint8_t));
                std::memcpy(&user3, &output_cloud.data[i * output_cloud.point_step + userField.offset + 2], sizeof(uint8_t));
                cv::Vec3b pixel;
                pixel[0] = user3; // B
                pixel[1] = user2; // G
                pixel[2] = user1; // R
                userImage.image.at<cv::Vec3b>(j, k) = pixel;
              }
            }
            else if((userField.datatype == sensor_msgs::msg::PointField::UINT16  && userField.count == 4) ||
                    (userField.datatype == sensor_msgs::msg::PointField::FLOAT64 && (userField.name == "rgb" || userField.name == "rgba"))
                    )
            {
              // uint16_t user1, user2, user3;
              // std::memcpy(&user1, &output_cloud.data[i * output_cloud.point_step + userField.offset + 0],sizeof(uint16_t));
              // std::memcpy(&user2, &output_cloud.data[i * output_cloud.point_step + userField.offset + 1],sizeof(uint16_t));
              // std::memcpy(&user3, &output_cloud.data[i * output_cloud.point_step + userField.offset + 2],sizeof(uint16_t));
              // cv::Vec3w bgr_pixel;
              // bgr_pixel[0] = user1;
              // bgr_pixel[1] = user2;
              // bgr_pixel[2] = user3;
              // userImage.image.at<cv::Vec3w>(j, k) = bgr_pixel;

              // Handle 16-bit RGB/RGBA fields - fix offset calculations
              if (userField.datatype == sensor_msgs::msg::PointField::FLOAT64) {
                // 64-bit packed RGB as double
                double rgb_double;
                std::memcpy(&rgb_double, &output_cloud.data[i * output_cloud.point_step + userField.offset], sizeof(double));
                uint64_t rgb_packed = *reinterpret_cast<uint64_t*>(&rgb_double);
                uint16_t r = (rgb_packed >> 32) & 0xFFFF;
                uint16_t g = (rgb_packed >> 16) & 0xFFFF;
                uint16_t b = rgb_packed & 0xFFFF;
                cv::Vec3w bgr_pixel;
                bgr_pixel[0] = b; // B
                bgr_pixel[1] = g; // G
                bgr_pixel[2] = r; // R
                userImage.image.at<cv::Vec3w>(j, k) = bgr_pixel;
              } else {
                // 4 separate uint16 values (RGBA) - use proper sizeof() for offsets
                uint16_t user1, user2, user3;
                std::memcpy(&user1, &output_cloud.data[i * output_cloud.point_step + userField.offset + 0 * sizeof(uint16_t)], sizeof(uint16_t));
                std::memcpy(&user2, &output_cloud.data[i * output_cloud.point_step + userField.offset + 1 * sizeof(uint16_t)], sizeof(uint16_t));
                std::memcpy(&user3, &output_cloud.data[i * output_cloud.point_step + userField.offset + 2 * sizeof(uint16_t)], sizeof(uint16_t));
                cv::Vec3w bgr_pixel;
                bgr_pixel[0] = user3; // B
                bgr_pixel[1] = user2; // G
                bgr_pixel[2] = user1; // R
                userImage.image.at<cv::Vec3w>(j, k) = bgr_pixel;
              }
            }
            else if((userField.datatype == sensor_msgs::msg::PointField::UINT8))
            {
              
              uint8_t value;
              std::memcpy(&value, &output_cloud.data[i * output_cloud.point_step + userField.offset],sizeof(uint8_t));
              userImage.image.at<uint8_t>(j, k) = value;      
            }
            else if((userField.datatype == sensor_msgs::msg::PointField::INT8))
            {
              int8_t value;
              std::memcpy(&value, &output_cloud.data[i * output_cloud.point_step + userField.offset],sizeof(int8_t));
              userImage.image.at<int8_t>(j, k) = value;         
            }        
            else if((userField.datatype == sensor_msgs::msg::PointField::UINT16))
            {
              uint16_t value;
              std::memcpy(&value, &output_cloud.data[i * output_cloud.point_step + userField.offset],sizeof(uint16_t));
              userImage.image.at<uint16_t>(j, k) = value;          
            }
            else if((userField.datatype == sensor_msgs::msg::PointField::INT16))
            {
              int16_t value;
              std::memcpy(&value, &output_cloud.data[i * output_cloud.point_step + userField.offset],sizeof(int16_t));
              userImage.image.at<int16_t>(j, k) = value;
            }
            else if((userField.datatype == sensor_msgs::msg::PointField::FLOAT32))
            {
              float value;
              std::memcpy(&value, &output_cloud.data[i * output_cloud.point_step + userField.offset],sizeof(float));
              userImage.image.at<float>(j, k) = value;         
            }        
            else if((userField.datatype == sensor_msgs::msg::PointField::FLOAT64))
            {
              double value;
              std::memcpy(&value, &output_cloud.data[i * output_cloud.point_step + userField.offset],sizeof(double));
              userImage.image.at<double>(j, k) = value;
            }  
          }   
        } 
      }
    }
  }

  // MASK-BASED RENDERING: Step 4 - Final compositing
  // Composite the final image by blending data with gradient background
  // Where mask == 255 (has data): use actual data values
  // Where mask == 0 (no data): use gradient background values
  RCLCPP_DEBUG_STREAM(node_->get_logger(), "  Final Compositing");
  
  if(field_ == "depth") {
    for (int row = 0; row < height_; row++) {
      for (int col = 0; col < width_; col++) {
        if (depthMask.at<uint8_t>(row, col) == 255) {
          // Pixel has real depth data - convert depth to grayscale and display as white/gray
          float depth_value = depthBuffer.at<float>(row, col);
          // Normalize depth to 0-255 range (assuming max depth ~10 meters)
          uint8_t intensity = static_cast<uint8_t>(std::min(255.0f, depth_value * 25.5f)); // 10m -> 255
          
          cv::Vec3b& pixel = depthImage.image.at<cv::Vec3b>(row, col);
          pixel[0] = intensity; // B
          pixel[1] = intensity; // G  
          pixel[2] = intensity; // R (grayscale)
        } else {
          // Pixel has no depth data - use gradient background value
          cv::Vec3b background_pixel = gradientBackground.at<cv::Vec3b>(row, col);
          depthImage.image.at<cv::Vec3b>(row, col) = background_pixel;
        }
      }
    }
  } else if(field_ == "intensity") {
    // Handle raw intensity field data with gradient background overlay
    RCLCPP_DEBUG_STREAM(node_->get_logger(), "Processing raw intensity field with gradient background");
    
    // Create intensity image with proper data type preservation
    cv::Mat intensityImage;
    cv::Mat intensityMask = cv::Mat::zeros(height_, width_, CV_8UC1);
    std::string encoding;
    
    // Initialize intensity image based on field data type to preserve precision
    if (userField.datatype == sensor_msgs::msg::PointField::UINT8) {
      intensityImage = cv::Mat::zeros(height_, width_, CV_8UC1);
      encoding = "mono8";
    } else if (userField.datatype == sensor_msgs::msg::PointField::UINT16) {
      intensityImage = cv::Mat::zeros(height_, width_, CV_16UC1);
      encoding = "mono16";
    } else if (userField.datatype == sensor_msgs::msg::PointField::FLOAT32) {
      intensityImage = cv::Mat::zeros(height_, width_, CV_32FC1);
      encoding = "32FC1";
    } else if (userField.datatype == sensor_msgs::msg::PointField::FLOAT64) {
      intensityImage = cv::Mat::zeros(height_, width_, CV_64FC1);
      encoding = "64FC1";
    } else {
      // Default to 8-bit if unsupported type
      intensityImage = cv::Mat::zeros(height_, width_, CV_8UC1);
      encoding = "mono8";
    }
    
    // Iterate through the point cloud and extract intensity data
    for (size_t i = 0; i < obj_pts.size(); ++i) {
      int u = static_cast<int>(img_pts[i].x);
      int v = static_cast<int>(img_pts[i].y);

      if (u >= 0 && u < width_ && v >= 0 && v < height_) {
        
        // Buffer bounds checking for intensity field data access
        // Skip if userField is invalid
        // COMMENTED OUT: This validation might filter out valid lidar intensity data with extended field types
        // if (userField.datatype == 0 || userField.datatype > 8) {
        //   continue; // Skip this point if userField is invalid
        // }
        
        size_t point_start = i * output_cloud.point_step;
        size_t intensity_field_access = point_start + userField.offset;
        size_t intensity_field_size = PointCloud2Subscriber::sizeOfPointField(userField.datatype);
        
        // Validate intensity field buffer bounds
        if (intensity_field_access + intensity_field_size > output_cloud.data.size()) {
          RCLCPP_WARN_STREAM_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000, 
                                       "Intensity field buffer access out of bounds for point " << i << ", skipping point");
          continue;
        }
        
        // Extract raw intensity value preserving original data type
        if (userField.datatype == sensor_msgs::msg::PointField::UINT8) {
          uint8_t intensity_value;
          std::memcpy(&intensity_value, &output_cloud.data[i * output_cloud.point_step + userField.offset], sizeof(uint8_t));
          intensityImage.at<uint8_t>(v, u) = intensity_value;
          intensityMask.at<uint8_t>(v, u) = 255;
        } else if (userField.datatype == sensor_msgs::msg::PointField::UINT16) {
          uint16_t intensity_value;
          std::memcpy(&intensity_value, &output_cloud.data[i * output_cloud.point_step + userField.offset], sizeof(uint16_t));
          intensityImage.at<uint16_t>(v, u) = intensity_value;
          intensityMask.at<uint8_t>(v, u) = 255;
        } else if (userField.datatype == sensor_msgs::msg::PointField::FLOAT32) {
          float intensity_value;
          std::memcpy(&intensity_value, &output_cloud.data[i * output_cloud.point_step + userField.offset], sizeof(float));
          intensityImage.at<float>(v, u) = intensity_value;
          intensityMask.at<uint8_t>(v, u) = 255;
        } else if (userField.datatype == sensor_msgs::msg::PointField::FLOAT64) {
          double intensity_value;
          std::memcpy(&intensity_value, &output_cloud.data[i * output_cloud.point_step + userField.offset], sizeof(double));
          intensityImage.at<double>(v, u) = intensity_value;
          intensityMask.at<uint8_t>(v, u) = 255;
        }
        
        // Debug: Log intensity values occasionally
        if (i % 1000 == 0) {
          RCLCPP_DEBUG_STREAM(node_->get_logger(), "Point " << i << " processed for intensity");
        }
      }
    }
    
    // For visualization, convert to BGR with gradient background overlay
    cv::Mat intensityBGR = cv::Mat::zeros(height_, width_, CV_8UC3);
    
    // Apply gradient background where no intensity data exists
    for (int row = 0; row < height_; row++) {
      for (int col = 0; col < width_; col++) {
        if (intensityMask.at<uint8_t>(row, col) == 255) {
          // Has intensity data - convert to grayscale for display
          uint8_t display_value = 128; // Default
          
          if (userField.datatype == sensor_msgs::msg::PointField::UINT8) {
            display_value = intensityImage.at<uint8_t>(row, col);
          } else if (userField.datatype == sensor_msgs::msg::PointField::UINT16) {
            // Scale 16-bit to 8-bit for display
            display_value = static_cast<uint8_t>(intensityImage.at<uint16_t>(row, col) >> 8);
          } else if (userField.datatype == sensor_msgs::msg::PointField::FLOAT32) {
            // Clamp float to 0-255 range
            float val = intensityImage.at<float>(row, col);
            display_value = static_cast<uint8_t>(std::min(255.0f, std::max(0.0f, val)));
          } else if (userField.datatype == sensor_msgs::msg::PointField::FLOAT64) {
            // Clamp double to 0-255 range
            double val = intensityImage.at<double>(row, col);
            display_value = static_cast<uint8_t>(std::min(255.0, std::max(0.0, val)));
          }
          
          // Set as grayscale (intensity data)
          intensityBGR.at<cv::Vec3b>(row, col) = cv::Vec3b(display_value, display_value, display_value);
        } else {
          // No intensity data - use gradient background
          float saturation_ratio = 1.0f - (float)row / (float)height_;
          uint8_t blue_value = (uint8_t)(200 * saturation_ratio);
          uint8_t green_value = (uint8_t)(80 * saturation_ratio);
          uint8_t red_value = (uint8_t)(0 * saturation_ratio);
          intensityBGR.at<cv::Vec3b>(row, col) = cv::Vec3b(blue_value, green_value, red_value);
        }
      }
    }
    
    // Convert and publish intensity image with gradient background
    sensor_msgs::msg::Image output_msg;
    std_msgs::msg::Header header;
    header.stamp = output_cloud.header.stamp;
    header.frame_id = frame_id_;
    cv_bridge::CvImage(header, "bgr8", intensityBGR).toImageMsg(output_msg);
    sensor_msgs::msg::Image::ConstPtr output_ptr = std::make_shared<sensor_msgs::msg::Image>(output_msg);
    callback_(output_ptr);
    return;
  } else {
    // For other fields (intensity, etc.), apply gradient background where no data exists
    if(userImage.encoding == sensor_msgs::image_encodings::TYPE_8UC3) {
      for (int row = 0; row < height_; row++) {
        for (int col = 0; col < width_; col++) {
          if (depthMask.at<uint8_t>(row, col) == 0) {
            // No data at this pixel - use medium blue saturation-based gradient background
            // Medium blue color: RGB(0, 80, 200) -> BGR(200, 80, 0)
            float saturation_ratio = 1.0f - (float)row / (float)height_;
            uint8_t blue_value = (uint8_t)(200 * saturation_ratio);
            uint8_t green_value = (uint8_t)(80 * saturation_ratio);
            uint8_t red_value = (uint8_t)(0 * saturation_ratio);
            cv::Vec3b& pixel = userImage.image.at<cv::Vec3b>(row, col);
            pixel[0] = blue_value; // B
            pixel[1] = green_value; // G
            pixel[2] = red_value;   // R
          }
        }
      }
    } else if(userImage.encoding == sensor_msgs::image_encodings::TYPE_16UC3) {
      for (int row = 0; row < height_; row++) {
        for (int col = 0; col < width_; col++) {
          if (depthMask.at<uint8_t>(row, col) == 0) {
            // No data at this pixel - use medium blue saturation-based gradient background
            // Medium blue color scaled to 16-bit: RGB(0, 80, 200) -> BGR(51400, 20560, 0)
            float saturation_ratio = 1.0f - (float)row / (float)height_;
            uint16_t blue_value = (uint16_t)(51400 * saturation_ratio);
            uint16_t green_value = (uint16_t)(20560 * saturation_ratio);
            uint16_t red_value = (uint16_t)(0 * saturation_ratio);
            cv::Vec3w& pixel = userImage.image.at<cv::Vec3w>(row, col);
            pixel[0] = blue_value; // B
            pixel[1] = green_value; // G
            pixel[2] = red_value;   // R
          }
          // Point cloud data takes priority - no modification needed for pixels with data
        }
      }
    } else if(userImage.encoding == sensor_msgs::image_encodings::TYPE_8UC1 ||
              userImage.encoding == sensor_msgs::image_encodings::TYPE_8SC1 ||
              userImage.encoding == sensor_msgs::image_encodings::TYPE_16UC1 ||
              userImage.encoding == sensor_msgs::image_encodings::TYPE_16SC1 ||
              userImage.encoding == sensor_msgs::image_encodings::TYPE_32FC1 ||
              userImage.encoding == sensor_msgs::image_encodings::TYPE_64FC1) {
      // Create type-appropriate gradient directly in the original image format
      // This preserves the data range and avoids type conversion issues
      
      for (int row = 0; row < height_; row++) {
        for (int col = 0; col < width_; col++) {
          if (depthMask.at<uint8_t>(row, col) == 0) {
            // Pixel has no data - fill with type-appropriate gradient value
            float saturation_ratio = 1.0f - (float)row / (float)height_; // 1.0 at top, 0.0 at bottom
            
            if (userImage.encoding == sensor_msgs::image_encodings::TYPE_8UC1) {
              // For 8-bit unsigned: gradient from 200 at top to 0 at bottom
              uint8_t gradient_value = (uint8_t)(200 * saturation_ratio);
              userImage.image.at<uint8_t>(row, col) = gradient_value;
            } else if (userImage.encoding == sensor_msgs::image_encodings::TYPE_8SC1) {
              // For 8-bit signed: gradient from 100 at top to -100 at bottom
              int8_t gradient_value = (int8_t)(100 * (2.0f * saturation_ratio - 1.0f));
              userImage.image.at<int8_t>(row, col) = gradient_value;
            } else if (userImage.encoding == sensor_msgs::image_encodings::TYPE_16UC1) {
              // For 16-bit unsigned: gradient from 51400 at top to 0 at bottom
              uint16_t gradient_value = (uint16_t)(51400 * saturation_ratio);
              userImage.image.at<uint16_t>(row, col) = gradient_value;
            } else if (userImage.encoding == sensor_msgs::image_encodings::TYPE_16SC1) {
              // For 16-bit signed: gradient from 25700 at top to -25700 at bottom  
              int16_t gradient_value = (int16_t)(25700 * (2.0f * saturation_ratio - 1.0f));
              userImage.image.at<int16_t>(row, col) = gradient_value;
            } else if (userImage.encoding == sensor_msgs::image_encodings::TYPE_32FC1) {
              // For 32-bit float: gradient from 1.0 at top to 0.0 at bottom
              float gradient_value = saturation_ratio;
              userImage.image.at<float>(row, col) = gradient_value;
            } else if (userImage.encoding == sensor_msgs::image_encodings::TYPE_64FC1) {
              // For 64-bit double: gradient from 1.0 at top to 0.0 at bottom
              double gradient_value = (double)saturation_ratio;
              userImage.image.at<double>(row, col) = gradient_value;
            }
          }
          // Pixels with real data (depthMask == 255) are left unchanged
        }
      }
      
      // Use the original user image (now with gradient background) for output
      sensor_msgs::msg::Image output_msg;
      userImage.toImageMsg(output_msg);
      sensor_msgs::msg::Image::ConstPtr output_ptr = std::make_shared<sensor_msgs::msg::Image>(output_msg);
      callback_(output_ptr);
      return;
    }
  }

  // Performance timing
  auto endTime = std::chrono::steady_clock::now();
  auto totalTime = std::chrono::duration_cast<std::chrono::microseconds>(endTime - beginTime);
  double timeMS = totalTime.count() / 1000.0;
  RCLCPP_DEBUG_STREAM(node_->get_logger(), "Processing time: " << timeMS << "ms");
  sensor_msgs::msg::Image output_msg;
  if(field_ == "depth") depthImage.toImageMsg(output_msg);
  else userImage.toImageMsg(output_msg);

  sensor_msgs::msg::Image::ConstPtr output_ptr = std::make_shared<sensor_msgs::msg::Image>(output_msg);
  callback_(output_ptr);

  return;
}

bool PointCloud2Subscriber::compareFieldsOffset(sensor_msgs::msg::PointField& field1, sensor_msgs::msg::PointField& field2)
{
  return (field1.offset < field2.offset);
}

std::shared_ptr<RosSubscriber> PointCloud2SubscriberType::create_subscriber(rclcpp::Node::SharedPtr node)
{
  return std::shared_ptr<RosSubscriber>(
      new PointCloud2Subscriber(node));
}

}