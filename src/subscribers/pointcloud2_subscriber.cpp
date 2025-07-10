#include <math.h>
#include "web_video_server/subscribers/pointcloud2_subscriber.hpp"

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
      
      // Transform into a z-forward orientation for opencv
      transform_optical_.header = transform.header;
      tf2::doTransform(transform.transform, transform.transform, transform_optical_);
      tf2::doTransform(*input_msg, output_cloud, transform);
  }
  catch (tf2::TransformException &ex) 
  {
    RCLCPP_WARN_STREAM(node_->get_logger(),"  Publish Thread: " << ex.what());
    output_cloud = *input_msg;
    return;
  }

  //Sort input cloud fields by field offset
   RCLCPP_DEBUG_STREAM(node_->get_logger(),"  Sort Fields");
  std::vector<sensor_msgs::msg::PointField> sortedFields(input_msg->fields);
  std::sort(sortedFields.begin(), sortedFields.end(), compareFieldsOffset);

  // Find fields we need in the cloud]
  bool xFound = false, yFound = false, zFound = false;
  bool userFound = false;
  sensor_msgs::msg::PointField xField, yField, zField, userField;
  for (int i=0; i < sortedFields.size(); i++)
  {
    sensor_msgs::msg::PointField currentField = sortedFields[i];
    
    if(currentField.name == "x")
    {
      xFound = true;
      xField = currentField;
       RCLCPP_DEBUG_STREAM(node_->get_logger(),"   Found X Field");
    }
    else if(currentField.name == "y")
    {
      yFound = true;
      yField = currentField;
       RCLCPP_DEBUG_STREAM(node_->get_logger(),"   Found Y Field");            
    }
    else if(currentField.name == "z")
    {
      zFound = true;
      zField = currentField;
       RCLCPP_DEBUG_STREAM(node_->get_logger(),"   Found Z Field");            
    }
    if(currentField.name == field_)
    {
      userFound = true;
      userField = currentField;
       RCLCPP_DEBUG_STREAM(node_->get_logger(),"   Found " << currentField.name);             
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

  // Setup depth image
   RCLCPP_DEBUG_STREAM(node_->get_logger(),"    Depth");           
  cv_bridge::CvImage depthImage;
  depthImage.header = output_cloud.header;
  depthImage.header.frame_id = frame_id_;
  depthImage.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
  depthImage.image = cv::Mat::zeros(height_, width_, CV_32FC1);

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
    return;
  }

  // Setup OpenCV matrixes
   RCLCPP_DEBUG_STREAM(node_->get_logger(),"  Setup opencv matrixes");        
  cv::Mat rvec = cv::Mat::zeros(3, 1, cv::DataType<double>::type);
  cv::Mat tvec = cv::Mat::zeros(3, 1, cv::DataType<double>::type);
  std::vector<cv::Point3f> obj_pts;
  std::vector<cv::Point2f> img_pts;

  // #pragma omp parallel for
  int size = output_cloud.height * output_cloud.width;
   RCLCPP_DEBUG_STREAM(node_->get_logger(),"  Create depth image for opencv: " << size);
  for (int i = 0; i < size; ++i)
  {
    float X,Y,Z;
    std::memcpy(&X, &output_cloud.data[i * output_cloud.point_step + xField.offset],sizeof(sizeOfPointField(yField.datatype)));
    std::memcpy(&Y, &output_cloud.data[i * output_cloud.point_step + yField.offset],sizeof(sizeOfPointField(yField.datatype)));
    std::memcpy(&Z, &output_cloud.data[i * output_cloud.point_step + zField.offset],sizeof(sizeOfPointField(zField.datatype)));
    obj_pts.push_back(cv::Point3f(X, Y, Z));
  }
  
  cv::projectPoints(obj_pts, rvec, tvec, intrinsicMatrix, distortionCoefficients, img_pts);

  // #pragma omp parallel for
  // Loop through points
   RCLCPP_DEBUG_STREAM(node_->get_logger(),"  Loop through points: " << img_pts.size());        
  for (size_t i = 0; i < img_pts.size(); ++i)
  {
    int u = int(img_pts[i].x);
    int v = int(img_pts[i].y);
    
    // Check if point is inside fov of camera
    if ((u >= 0) && (u < depthImage.image.cols) &&  
        (v >= 0) && (v < depthImage.image.rows) &&
        obj_pts[i].z > 0)
    {
      // add to depth image
      if(depthImage.image.at<float>(v, u) <= 0.001 || 
        depthImage.image.at<float>(v, u) > obj_pts[i].z)
      {
        // color values
        // uint8_t b,g,r;
        // std::memcpy(&b, &output_cloud.data[i * output_cloud.point_step + rgbField.offset + 0],sizeof(uint8_t));
        // std::memcpy(&g, &output_cloud.data[i * output_cloud.point_step + rgbField.offset + 1],sizeof(uint8_t));
        // std::memcpy(&r, &output_cloud.data[i * output_cloud.point_step + rgbField.offset + 2],sizeof(uint8_t));              
        
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
            // // color image
            // cv::Vec3b bgr_pixel;
            // bgr_pixel[0] = b;
            // bgr_pixel[1] = g;
            // bgr_pixel[2] = r;
            // colorImage.image.at<cv::Vec3b>(j, k) = bgr_pixel;

            //depth image
            depthImage.image.at<float>(j, k) = obj_pts[i].z;
            
            // user image
            if( (userField.datatype == sensor_msgs::msg::PointField::UINT8   && userField.count == 4) ||
                (userField.datatype == sensor_msgs::msg::PointField::UINT32  && (userField.name == "rgb" || userField.name == "rgba")) ||
                (userField.datatype == sensor_msgs::msg::PointField::FLOAT32 && (userField.name == "rgb" || userField.name == "rgba"))
              )
            {
              uint8_t user1, user2, user3;
              std::memcpy(&user1, &output_cloud.data[i * output_cloud.point_step + userField.offset + 0],sizeof(uint8_t));
              std::memcpy(&user2, &output_cloud.data[i * output_cloud.point_step + userField.offset + 1],sizeof(uint8_t));
              std::memcpy(&user3, &output_cloud.data[i * output_cloud.point_step + userField.offset + 2],sizeof(uint8_t));
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
              std::memcpy(&user1, &output_cloud.data[i * output_cloud.point_step + userField.offset + 0],sizeof(uint16_t));
              std::memcpy(&user2, &output_cloud.data[i * output_cloud.point_step + userField.offset + 1],sizeof(uint16_t));
              std::memcpy(&user3, &output_cloud.data[i * output_cloud.point_step + userField.offset + 2],sizeof(uint16_t));
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

  // Check timer
  auto endTime = std::chrono::steady_clock::now();
  auto totalTime = endTime - beginTime;
  double timeMS = totalTime.count() / 1000.0;
  RCLCPP_DEBUG_STREAM(node_->get_logger(), " timer: " << timeMS);

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

std::shared_ptr<RosSubscriber> PointCloud2SubscriberType::create_subscriber(rclcpp::Node::SharedPtr node)
{
  return std::shared_ptr<RosSubscriber>(
      new PointCloud2Subscriber(node));
}


}