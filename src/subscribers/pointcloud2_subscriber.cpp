#include "web_video_server/subscribers/pointcloud2_subscriber.hpp"

namespace web_video_server
{

PointCloud2Subscriber::PointCloud2Subscriber(rclcpp::Node::SharedPtr node)
: RosSubscriber(node)
{
  std::scoped_lock lock(subscriber_mutex_);
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
    std::scoped_lock lock(subscriber_mutex_);
}

void PointCloud2Subscriber::subscribe(const async_web_server_cpp::HttpRequest &request,
                                         const std::string& topic, 
                                         const ImageCallback& callback)
{
  std::scoped_lock lock(subscriber_mutex_);
  callback_ = callback;
  qos_profile_name_ = request.get_query_param_value_or_default("qos_profile", "default");

  frame_id_ = request.get_query_param_value_or_default("frame_id", "base_link");
  wait_for_tf_delay_ = 0.10;
  field_ = request.get_query_param_value_or_default("field", "depth");
  height_ = request.get_query_param_value_or_default<int>("height", 600);
  width_  = request.get_query_param_value_or_default<int>("width", 800);
  pixel_size_ = request.get_query_param_value_or_default<int>("pixel_size", 5);
  focal_length_ = request.get_query_param_value_or_default<double>("focal_length", 300.0);
  background_ = request.get_query_param_value_or_default("background", true);  

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

void PointCloud2Subscriber::subscriberCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &input_msg)
{
  RCLCPP_DEBUG_STREAM(node_->get_logger(),"Received PointCloud2: ");
  std::scoped_lock lock(subscriber_mutex_);

  if (input_msg->data.size() == 0)
  {
    RCLCPP_WARN_STREAM_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000, "No data in pointcloud!");
    return;
  }

  // Start timer
  auto beginTime = std::chrono::steady_clock::now();

  //transform
  sensor_msgs::msg::PointCloud2 output_cloud = TransformFrame(input_msg, frame_id_);

  // Find relevant fields
  sensor_msgs::msg::PointField xField, yField, zField, userField;  
  userField.name = field_;
  if(!FindFields(input_msg, userField, xField, yField, zField)) return;

  // Setup camera_info
  RCLCPP_DEBUG_STREAM(node_->get_logger(),"    Camera Info");
  cv::Mat intrinsic_matrix, distortion_coefficients;
  GatherCameraInfo(intrinsic_matrix, distortion_coefficients);
  // Setup depth image
  RCLCPP_DEBUG_STREAM(node_->get_logger(),"    Depth");           
  cv_bridge::CvImage depthImage;
  depthImage.header = output_cloud.header;
  depthImage.header.frame_id = frame_id_;
  depthImage.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
  depthImage.image = cv::Mat::ones(height_, width_, CV_32FC1) * std::numeric_limits<float>::max();

  // Setup user image
  RCLCPP_DEBUG_STREAM(node_->get_logger(),"    User-Datatype: " << +userField.datatype);
  cv_bridge::CvImage userImage;
  if(!CreateUserImage(output_cloud.header, userField, userImage)) return;
  
  // Project Points
   RCLCPP_DEBUG_STREAM(node_->get_logger(),"  Project points");        
  std::vector<cv::Point3f> obj_pts;
  std::vector<cv::Point2f> img_pts = ProjectPoints(output_cloud, xField, yField, zField, intrinsic_matrix, distortion_coefficients, obj_pts);

  // Process projected points and render to image
  cv::Mat depthMask = cv::Mat::zeros(height_, width_, CV_8UC1);
  // Loop through points and fillout user and depth images
  RCLCPP_DEBUG_STREAM(node_->get_logger(),"  Process projected points: " << img_pts.size());     
  for (size_t i = 0; i < img_pts.size(); ++i)
  {
    int u = int(img_pts[i].x);
    int v = int(img_pts[i].y);

    // Check if point is inside field of view and has valid projection
    // Filter out rear points that are incorrectly projected into front view
    // Keep points that are properly in front of sensor for current camera view
    // RELAXED BOUNDS: Allow points slightly outside FOV to capture more lidar data
    if ((u >= -pixel_size_) && (u < width_ + pixel_size_) &&  
        (v >= -pixel_size_) && (v < height_ + pixel_size_) &&
        obj_pts[i].z > 0.1)  // Filter out points behind/very close to sensor that cause incorrect projection
    {
      // update depth image
      if(depthImage.image.at<float>(v, u) > obj_pts[i].z)
      {
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
            // Update the depth image if point is closer to camera
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
              std::memcpy(&user1, &output_cloud.data[i * output_cloud.point_step + userField.offset + 0], sizeof(uint8_t));
              std::memcpy(&user2, &output_cloud.data[i * output_cloud.point_step + userField.offset + 1], sizeof(uint8_t));
              std::memcpy(&user3, &output_cloud.data[i * output_cloud.point_step + userField.offset + 2], sizeof(uint8_t));
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
              std::memcpy(&user1, &output_cloud.data[i * output_cloud.point_step + userField.offset + 0], sizeof(uint16_t));
              std::memcpy(&user2, &output_cloud.data[i * output_cloud.point_step + userField.offset + 2], sizeof(uint16_t));
              std::memcpy(&user3, &output_cloud.data[i * output_cloud.point_step + userField.offset + 4], sizeof(uint16_t));
              cv::Vec3w bgr_pixel;
              bgr_pixel[0] = user1;
              bgr_pixel[1] = user2;
              bgr_pixel[2] = user3;
              userImage.image.at<cv::Vec3w>(j, k) = bgr_pixel;
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

  cv_bridge::CvImage colorImage;
  if (background_) {
    colorImage = ConvertToColor(depthMask, depthImage, userImage);
  } else if (field_ == "depth") {
    colorImage = depthImage;
  } else {
    colorImage = userImage;
  }

  // Performance timing
  auto endTime = std::chrono::steady_clock::now();
  auto totalTime = endTime - beginTime;
  auto timeMS = std::chrono::duration_cast<std::chrono::milliseconds>(totalTime);
  RCLCPP_DEBUG_STREAM(node_->get_logger(), "Processing time: " << timeMS.count() << "ms");
  sensor_msgs::msg::Image output_msg;
  colorImage.toImageMsg(output_msg);
  sensor_msgs::msg::Image::ConstSharedPtr output_ptr = std::make_shared<sensor_msgs::msg::Image>(output_msg);
  callback_(output_ptr);

  return;
}

bool PointCloud2Subscriber::compareFieldsOffset(sensor_msgs::msg::PointField& field1, sensor_msgs::msg::PointField& field2)
{
  return (field1.offset < field2.offset);
}

inline int sizeOfPointField(int datatype)
{
  if ((datatype == sensor_msgs::msg::PointField::INT8) || (datatype == sensor_msgs::msg::PointField::UINT8))
    return 1;
  else if ((datatype == sensor_msgs::msg::PointField::INT16) || (datatype == sensor_msgs::msg::PointField::UINT16))
    return 2;
  else if ((datatype == sensor_msgs::msg::PointField::INT32) || (datatype == sensor_msgs::msg::PointField::UINT32) ||
           (datatype == sensor_msgs::msg::PointField::FLOAT32))
    return 4;
  else if (datatype == sensor_msgs::msg::PointField::FLOAT64)
    return 8;
  else
  {
    std::stringstream err;
    err << "PointField of type " << datatype << " does not exist";
    throw std::runtime_error(err.str());
  }
  return -1;
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
    RCLCPP_WARN_STREAM(node_->get_logger(),"Cloud does not contain XYZ data!");
    return false;
  }
  else if(xField.datatype != 7 || yField.datatype != 7 || zField.datatype != 7)
  {
    RCLCPP_WARN_STREAM(node_->get_logger(),"X, Y, Z fields do not contain floats!");
    return false;
  }

  return true;
}

sensor_msgs::msg::PointCloud2 PointCloud2Subscriber::TransformFrame(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &input_msg, std::string frame_id)
{
  RCLCPP_DEBUG_STREAM(node_->get_logger(),"  Transform");
  geometry_msgs::msg::TransformStamped transform;
  sensor_msgs::msg::PointCloud2 output_cloud;  
  try
  {
    //Get transform to requested frame
    transform = tf_buffer_->lookupTransform(frame_id, input_msg->header.frame_id, input_msg->header.stamp, rclcpp::Duration::from_seconds(wait_for_tf_delay_));
      
    // Transform into a z-forward orientation of requested frame for opencv
    transform_optical_.header = transform.header;
    tf2::doTransform(transform.transform, transform.transform, transform_optical_);
    tf2::doTransform(*input_msg, output_cloud, transform);
  }
  catch (tf2::TransformException &ex) 
  {
    RCLCPP_WARN_STREAM(node_->get_logger(),"  Publish Thread: " << ex.what());
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
  intrinsic_matrix.at<double>(0,2) = width_/2;
  intrinsic_matrix.at<double>(1,2) = height_/2;
  intrinsic_matrix.at<double>(2,2) = 1.0;
}

bool PointCloud2Subscriber::CreateUserImage(const std_msgs::msg::Header &cloud_header, const sensor_msgs::msg::PointField &userField, cv_bridge::CvImage& userImage)
{ 
  userImage.header = cloud_header;
  userImage.header.frame_id = frame_id_;
  if((userField.datatype == sensor_msgs::msg::PointField::UINT8   && userField.count == 4) ||
      (userField.datatype == sensor_msgs::msg::PointField::UINT32  && (userField.name == "rgb" || userField.name == "rgba")) ||
      (userField.datatype == sensor_msgs::msg::PointField::FLOAT32 && (userField.name == "rgb" || userField.name == "rgba"))
    )
  {
     RCLCPP_DEBUG_STREAM(node_->get_logger(),"      4 8-bit unsigned integers");       
    userImage.encoding = sensor_msgs::image_encodings::TYPE_8UC3;
    userImage.image = cv::Mat::zeros(height_, width_, CV_8UC3);
  }
  else if((userField.datatype == sensor_msgs::msg::PointField::UINT16  && userField.count == 4) ||
          (userField.datatype == sensor_msgs::msg::PointField::FLOAT64 && (userField.name == "rgb" || userField.name == "rgba"))
          )
  {
     RCLCPP_DEBUG_STREAM(node_->get_logger(),"      4 16-bit unsigned integers");    
    userImage.encoding = sensor_msgs::image_encodings::TYPE_16UC3;
    userImage.image = cv::Mat::zeros(height_, width_, CV_16UC3);
  }
  else if((userField.datatype == sensor_msgs::msg::PointField::UINT8))
  {
     RCLCPP_DEBUG_STREAM(node_->get_logger(),"      1 8-bit unsigned integer");    
    userImage.encoding = sensor_msgs::image_encodings::TYPE_8UC1;
    userImage.image = cv::Mat::zeros(height_, width_, CV_8UC1);         
  }
  else if((userField.datatype == sensor_msgs::msg::PointField::INT8))
  {
     RCLCPP_DEBUG_STREAM(node_->get_logger(),"      1 8-bit signed integer");             
    userImage.encoding = sensor_msgs::image_encodings::TYPE_8SC1;
    userImage.image = cv::Mat::zeros(height_, width_, CV_8SC1);            
  }        
  else if((userField.datatype == sensor_msgs::msg::PointField::UINT16))
  {
     RCLCPP_DEBUG_STREAM(node_->get_logger(),"      1 16-bit unsigned integer");               
    userImage.encoding = sensor_msgs::image_encodings::TYPE_16UC1;
    userImage.image = cv::Mat::zeros(height_, width_, CV_16UC1);            
  }
  else if((userField.datatype == sensor_msgs::msg::PointField::INT16))
  {
     RCLCPP_DEBUG_STREAM(node_->get_logger(),"      1 16-bit signed integer");               
    userImage.encoding = sensor_msgs::image_encodings::TYPE_16SC1;
    userImage.image = cv::Mat::zeros(height_, width_, CV_16SC1);  
  }
  else if((userField.datatype == sensor_msgs::msg::PointField::FLOAT32))
  {
     RCLCPP_DEBUG_STREAM(node_->get_logger(),"      1 32-bit float");     
    userImage.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
    userImage.image = cv::Mat::zeros(height_, width_, CV_32FC1);            
  }        
  else if((userField.datatype == sensor_msgs::msg::PointField::FLOAT64))
  {
     RCLCPP_DEBUG_STREAM(node_->get_logger(),"      1 64-bit float");               
    userImage.encoding = sensor_msgs::image_encodings::TYPE_64FC1;
    userImage.image = cv::Mat::zeros(height_, width_, CV_64FC1);  
  }        
  else if(field_ != "depth")
  {
    RCLCPP_WARN_STREAM(node_->get_logger(), "Requested field '" << field_ << "' not found in point cloud!");
    return false;
  }
  return true;
}

bool PointCloud2Subscriber::CreateDepthImage(const std_msgs::msg::Header &cloud_header, const sensor_msgs::msg::PointField &userField, cv_bridge::CvImage& depthImage)
{ 
}

std::vector<cv::Point2f> PointCloud2Subscriber::ProjectPoints(const sensor_msgs::msg::PointCloud2 &output_cloud,
      const sensor_msgs::msg::PointField &xField, const sensor_msgs::msg::PointField &yField, const sensor_msgs::msg::PointField &zField, 
      const cv::Mat &intrinsic_matrix, const cv::Mat &distortion_coefficients, std::vector<cv::Point3f> &obj_pts)
{
  // Create 3D points for projection
  int size = output_cloud.height * output_cloud.width;
  RCLCPP_DEBUG_STREAM(node_->get_logger(),"  Create 3D points for projection: " << size);
  obj_pts.reserve(size); // Reserve space for better performance

  for (int i = 0; i < size; ++i)
  {
    // Find index of xyz data
    size_t point_start = i * output_cloud.point_step;
    size_t x_access = point_start + xField.offset;
    size_t y_access = point_start + yField.offset;
    size_t z_access = point_start + zField.offset;
    
    // Validate buffer bounds for coordinate access using correct field sizes (fields previously confirmed to exist and be floats)
    if (x_access + 4 > output_cloud.data.size() ||
        y_access + 4 > output_cloud.data.size() ||
        z_access + 4 > output_cloud.data.size()) {
      RCLCPP_WARN_STREAM_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000, 
                                   "Buffer access out of bounds for point " << i << ", skipping point");
      continue;
    }
    
    // Extract X, Y, Z coordinates
    float X,Y,Z;
    std::memcpy(&X, &output_cloud.data[x_access], sizeof(float));
    std::memcpy(&Y, &output_cloud.data[y_access], sizeof(float));
    std::memcpy(&Z, &output_cloud.data[z_access], sizeof(float));
    obj_pts.push_back(cv::Point3f(X, Y, Z));
  }
  
  cv::Mat rvec = cv::Mat::zeros(3, 1, cv::DataType<double>::type);
  cv::Mat tvec = cv::Mat::zeros(3, 1, cv::DataType<double>::type); 
  std::vector<cv::Point2f> img_pts;
  cv::projectPoints(obj_pts, rvec, tvec, intrinsic_matrix, distortion_coefficients, img_pts);
  return img_pts;
}

cv_bridge::CvImage PointCloud2Subscriber::ConvertToColor(const cv::Mat &depthMask, const cv_bridge::CvImage &depthImage, const cv_bridge::CvImage &userImage)
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
  RCLCPP_DEBUG_STREAM(node_->get_logger(), "  Final Compositing");
  // Setup color image
   RCLCPP_DEBUG_STREAM(node_->get_logger(),"    Color");        
  cv_bridge::CvImage colorImage;
  colorImage.header = userImage.header;
  colorImage.header.frame_id = frame_id_;
  if(field_ == "depth") {
    colorImage.encoding = sensor_msgs::image_encodings::BGR8;
    colorImage.image = cv::Mat::zeros(height_, width_, 'bgr8');  
    for (int row = 0; row < height_; row++) {
      for (int col = 0; col < width_; col++) {
        if (depthMask.at<uint8_t>(row, col) == 255) {
          // Pixel has real depth data - convert depth to grayscale and display as white/gray
          float depth_value = depthImage.image.at<float>(row, col);
          // Normalize depth to 0-255 range (assuming max depth ~10 meters)
          uint8_t intensity = static_cast<uint8_t>(std::min(255.0f, depth_value * 25.5f)); // 10m -> 255
          
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
  } else if(userImage.encoding == sensor_msgs::image_encodings::TYPE_16UC3) {
    colorImage.encoding = sensor_msgs::image_encodings::BGR16;
    colorImage.image = cv::Mat::zeros(height_, width_, 'bgr16');  
    for (int row = 0; row < height_; row++) {
      for (int col = 0; col < width_; col++) {
        if (depthMask.at<uint8_t>(row, col) == 255) {
          colorImage.image.at<cv::Vec3w>(row, col) = userImage.image.at<cv::Vec3w>(row, col);
        }
        else
        {
          // Pixel has no depth data - use gradient background value scaled to 16 bit
          colorImage.image.at<cv::Vec3w>(row, col) = gradientBackground.at<cv::Vec3b>(row, col) * 257;
        }
      }
    }
  } else if(userImage.encoding == sensor_msgs::image_encodings::TYPE_8UC3) {
    colorImage.encoding = sensor_msgs::image_encodings::BGR8;
    colorImage.image = cv::Mat::zeros(height_, width_, 'bgr8');      
    for (int row = 0; row < height_; row++) {
      for (int col = 0; col < width_; col++) {
        if (depthMask.at<uint8_t>(row, col) == 255) {
          colorImage.image.at<cv::Vec3b>(row, col) = userImage.image.at<cv::Vec3b>(row, col);
        } else {
          // Pixel has no depth data - use gradient background value
          colorImage.image.at<cv::Vec3b>(row, col) = gradientBackground.at<cv::Vec3b>(row, col);
        }
      }
    }    
  } else if(userImage.encoding == sensor_msgs::image_encodings::TYPE_8UC1) {
    colorImage.encoding = sensor_msgs::image_encodings::BGR8;
    colorImage.image = cv::Mat::zeros(height_, width_, 'bgr8');      
    for (int row = 0; row < height_; row++) {
      for (int col = 0; col < width_; col++) {
        if (depthMask.at<uint8_t>(row, col) == 255) {
          // Pixel has real depth data - convert depth to grayscale and display as white/gray
          float depth_value = userImage.image.at<uint8_t>(row, col);
          // Normalize depth to 0-255 range (assuming max depth ~10 meters)
          uint8_t intensity =depth_value; // 10m -> 255
          
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
  } else if(userImage.encoding == sensor_msgs::image_encodings::TYPE_8SC1) {
    colorImage.encoding = sensor_msgs::image_encodings::BGR8;
    colorImage.image = cv::Mat::zeros(height_, width_, 'bgr8');      
    for (int row = 0; row < height_; row++) {
      for (int col = 0; col < width_; col++) {
        if (depthMask.at<uint8_t>(row, col) == 255) {
          // Pixel has real depth data - convert depth to grayscale and display as white/gray
          int8_t depth_value = userImage.image.at<int8_t>(row, col);
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
  } else if(userImage.encoding == sensor_msgs::image_encodings::TYPE_16UC1) {
    colorImage.encoding = sensor_msgs::image_encodings::BGR16;
    colorImage.image = cv::Mat::zeros(height_, width_, 'bgr16'); 
    for (int row = 0; row < height_; row++) {
      for (int col = 0; col < width_; col++) {
        if (depthMask.at<uint8_t>(row, col) == 255) {
          // Pixel has real depth data - convert depth to grayscale and display as white/gray
          uint16_t depth_value = userImage.image.at<uint16_t>(row, col);
          // Normalize depth to 0-255 range
          uint16_t intensity = static_cast<uint16_t>(depth_value);
          
          cv::Vec3w& pixel = colorImage.image.at<cv::Vec3w>(row, col);
          pixel[0] = intensity; // B
          pixel[1] = intensity; // G  
          pixel[2] = intensity; // R (grayscale)
        } else {
          // Pixel has no depth data - use gradient background value
          colorImage.image.at<cv::Vec3w>(row, col) = gradientBackground.at<cv::Vec3b>(row, col) * 257;
        }
      }
    }
  } else if(userImage.encoding == sensor_msgs::image_encodings::TYPE_16SC1) {
    for (int row = 0; row < height_; row++) {
      for (int col = 0; col < width_; col++) {
        if (depthMask.at<uint8_t>(row, col) == 255) {
          // Pixel has real depth data - convert depth to grayscale and display as white/gray
          int16_t depth_value = userImage.image.at<int16_t>(row, col);
          // Normalize depth to 0-255 range
          uint16_t intensity = static_cast<uint16_t>(depth_value + 32768);
          
          cv::Vec3w& pixel = colorImage.image.at<cv::Vec3w>(row, col);
          pixel[0] = intensity; // B
          pixel[1] = intensity; // G  
          pixel[2] = intensity; // R (grayscale)
        } else {
          // Pixel has no depth data - use gradient background value
          colorImage.image.at<cv::Vec3w>(row, col) = gradientBackground.at<cv::Vec3b>(row, col) * 257;
        }
      }
    }    
  } else {
    colorImage = userImage;
  }
  
  return colorImage;
}

std::shared_ptr<RosSubscriber> PointCloud2SubscriberType::create_subscriber(rclcpp::Node::SharedPtr node)
{
  return std::shared_ptr<RosSubscriber>(
      new PointCloud2Subscriber(node));
}

}