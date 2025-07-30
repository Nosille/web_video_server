#include "web_video_server/rtsp_streamer.hpp"
#ifdef CV_BRIDGE_USES_OLD_HEADERS
#include <cv_bridge/cv_bridge.h>
#else
#include <cv_bridge/cv_bridge.hpp>
#endif
#include <sensor_msgs/image_encodings.hpp>
#include <gst/app/gstappsrc.h>
#include "async_web_server_cpp/http_request.hpp"
#include <algorithm>

namespace web_video_server
{

GstRTSPStreamer::GstRTSPStreamer(
    rclcpp::Node::SharedPtr node,
    const std::string & topic,
    const std::string & codec_name,
    int rtsp_port)
: node_(node),
  topic_(topic),
  codec_name_(codec_name),
  rtsp_port_(rtsp_port),
  active_(false)
{
    gst_init(NULL, NULL);
    subscriber_types_["image"] = std::make_shared<ImageTransportSubscriberType>();
}

GstRTSPStreamer::~GstRTSPStreamer()
{
    stop();
}

void GstRTSPStreamer::start()
{
    if (active_)
    {
        return;
    }

    main_loop_ = g_main_loop_new(NULL, FALSE);
    rtsp_server_ = gst_rtsp_server_new();
    g_object_set(rtsp_server_, "service", std::to_string(rtsp_port_).c_str(), NULL);

    GstRTSPMountPoints *mounts = gst_rtsp_server_get_mount_points(rtsp_server_);

    // Use appsrc to feed ROS camera data into the RTSP stream
    // Simplified pipeline with static configuration
    std::string pipeline_str = "( appsrc name=mysrc is-live=true do-timestamp=true ! videoconvert ! x264enc tune=zerolatency speed-preset=ultrafast bitrate=2000 ! rtph264pay name=pay0 pt=96 )";
    GstRTSPMediaFactory *factory = gst_rtsp_media_factory_new();
    gst_rtsp_media_factory_set_launch(factory, pipeline_str.c_str());
    gst_rtsp_media_factory_set_shared(factory, TRUE);
    
    // Set up media configure callback to access appsrc element
    g_signal_connect(factory, "media-configure", G_CALLBACK(+[](GstRTSPMediaFactory *factory, GstRTSPMedia *media, gpointer user_data) {
        GstRTSPStreamer *self = static_cast<GstRTSPStreamer*>(user_data);
        GstElement *element = gst_rtsp_media_get_element(media);
        self->appsrc_ = gst_bin_get_by_name(GST_BIN(element), "mysrc");
        
        if (self->appsrc_) {
            // Set appsrc properties for live streaming
            g_object_set(self->appsrc_, 
                "is-live", TRUE,
                "format", GST_FORMAT_TIME,
                "block", FALSE,  // Don't block to avoid flow issues
                "max-buffers", 1,  // Keep only 1 buffer to minimize latency
                NULL);
            // Caps will be set dynamically when first image arrives
        }
    }), this);
    
    // Track client connections to know when to push buffers
    g_signal_connect(factory, "media-constructed", G_CALLBACK(+[](GstRTSPMediaFactory *factory, GstRTSPMedia *media, gpointer user_data) {
        GstRTSPStreamer *self = static_cast<GstRTSPStreamer*>(user_data);
        g_signal_connect(media, "new-stream", G_CALLBACK(+[](GstRTSPMedia *media, GstRTSPStream *stream, gpointer user_data) {
            GstRTSPStreamer *self = static_cast<GstRTSPStreamer*>(user_data);
            self->has_clients_ = true;
        }), self);
        g_signal_connect(media, "removed-stream", G_CALLBACK(+[](GstRTSPMedia *media, GstRTSPStream *stream, gpointer user_data) {
            GstRTSPStreamer *self = static_cast<GstRTSPStreamer*>(user_data);
            self->has_clients_ = false;
        }), self);
    }), this);

    // Sanitize topic name for use as stream path - replace slashes with underscores
    std::string sanitized_topic = topic_;
    std::replace(sanitized_topic.begin(), sanitized_topic.end(), '/', '_');
    std::string stream_path = "/" + sanitized_topic;
    gst_rtsp_mount_points_add_factory(mounts, stream_path.c_str(), factory);
    g_object_unref(mounts);

    server_id_ = gst_rtsp_server_attach(rtsp_server_, NULL);

    gst_thread_ = std::thread([this]() {
        g_main_loop_run(main_loop_);
    });

    std::string subscriber_type = "image";
    if (subscriber_types_.find(subscriber_type) != subscriber_types_.end()) {
        subscriber_ = subscriber_types_[subscriber_type]->create_subscriber(node_);
        async_web_server_cpp::HttpRequest dummy_request;
        dummy_request.query = "qos_profile=default";
        subscriber_->subscribe(
            dummy_request,
            topic_,
            std::bind(&GstRTSPStreamer::imageCallback, this, std::placeholders::_1)
        );
    }

    active_ = true;
}

void GstRTSPStreamer::stop()
{
    if (!active_)
    {
        return;
    }

    if (main_loop_)
    {
        g_main_loop_quit(main_loop_);
        if (gst_thread_.joinable())
        {
            gst_thread_.join();
        }
        g_main_loop_unref(main_loop_);
        main_loop_ = nullptr;
    }

    if (rtsp_server_)
    {
        g_source_remove(server_id_);
        g_object_unref(rtsp_server_);
        rtsp_server_ = nullptr;
    }

    subscriber_.reset();
    active_ = false;
}

std::string GstRTSPStreamer::getStreamUrl() const
{
    // Sanitize topic name for RTSP path - replace slashes with underscores
    std::string sanitized_topic = topic_;
    std::replace(sanitized_topic.begin(), sanitized_topic.end(), '/', '_');
    return "rtsp://localhost:" + std::to_string(rtsp_port_) + "/" + sanitized_topic;
}

void GstRTSPStreamer::imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
{
    if (!active_ || !appsrc_)
    {
        return;
    }
    
    // Temporarily allow all buffer pushing for debugging
    // TODO: Re-enable client connection checking once pipeline is stable

cv_bridge::CvImagePtr cv_ptr;
    try
    {
        // Handle Bayer patterns
        if (msg->encoding == sensor_msgs::image_encodings::BAYER_RGGB8 || 
            msg->encoding == sensor_msgs::image_encodings::BAYER_BGGR8 || 
            msg->encoding == sensor_msgs::image_encodings::BAYER_GBRG8 || 
            msg->encoding == sensor_msgs::image_encodings::BAYER_GRBG8) {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        }
        // Handle color formats
        else if (msg->encoding == sensor_msgs::image_encodings::RGB8) {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        } 
        else if (msg->encoding == sensor_msgs::image_encodings::RGBA8) {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGRA8);
        } 
        else if (msg->encoding == sensor_msgs::image_encodings::YUV422) {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        }
        // Handle mono formats
        else if (msg->encoding == sensor_msgs::image_encodings::MONO8 || msg->encoding == sensor_msgs::image_encodings::MONO16) {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        }
        // Unsupported format
        else {
            RCLCPP_ERROR(node_->get_logger(), "Unsupported image encoding: %s", msg->encoding.c_str());
            return;
        }
    } catch (cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(node_->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    int width = cv_ptr->image.cols;
    int height = cv_ptr->image.rows;
    int channels = cv_ptr->image.channels();
    int size = width * height * channels;

    // Convert to BGR if we have BGRA (4 channels)
    if (channels == 4) {
        cv::cvtColor(cv_ptr->image, cv_ptr->image, cv::COLOR_BGRA2BGR);
        channels = 3;
        size = width * height * channels;
    }

    // Set caps dynamically based on first image
    if (!caps_set_) {
        GstCaps *caps = gst_caps_new_simple("video/x-raw",
            "format", G_TYPE_STRING, "BGR",
            "width", G_TYPE_INT, width,
            "height", G_TYPE_INT, height,
            "framerate", GST_TYPE_FRACTION, 30, 1,
            NULL);
        g_object_set(appsrc_, "caps", caps, NULL);
        gst_caps_unref(caps);
        caps_set_ = true;
        RCLCPP_INFO(node_->get_logger(), "RTSP stream caps set: %dx%d BGR from encoding %s", width, height, msg->encoding.c_str());
    }

    GstBuffer *buffer = gst_buffer_new_allocate(NULL, size, NULL);
    GstMapInfo map;
    gst_buffer_map(buffer, &map, GST_MAP_WRITE);
    memcpy(map.data, cv_ptr->image.data, size);
    gst_buffer_unmap(buffer, &map);

    GST_BUFFER_PTS(buffer) = frame_count_ * gst_util_uint64_scale_int(1, GST_SECOND, 30);
    GST_BUFFER_DURATION(buffer) = gst_util_uint64_scale_int(1, GST_SECOND, 30); // 30 fps
    frame_count_++;

    GstFlowReturn ret = gst_app_src_push_buffer(GST_APP_SRC(appsrc_), buffer);
    if (ret != GST_FLOW_OK)
    {
        // Don't log here, it's too noisy
    }
}

GstRTSPStreamerManager::GstRTSPStreamerManager(rclcpp::Node::SharedPtr node)
: node_(node), next_port_(8554)
{
    cleanup_timer_ = node_->create_wall_timer(
        std::chrono::seconds(5),
        std::bind(&GstRTSPStreamerManager::cleanupInactiveStreamers, this));
}

GstRTSPStreamerManager::~GstRTSPStreamerManager()
{
    cleanup();
}

std::shared_ptr<GstRTSPStreamer> GstRTSPStreamerManager::createStreamer(
    const std::string & topic,
    const std::string & codec,
    int rtsp_port)
{
    std::lock_guard<std::mutex> lock(streamers_mutex_);
    std::string stream_key = topic + "_" + codec;
    auto it = streamers_.find(stream_key);
    if (it != streamers_.end() && it->second->isActive())
    {
        return it->second;
    }

    if (rtsp_port == 0) {
        rtsp_port = next_port_++;
    }

    auto streamer = std::make_shared<GstRTSPStreamer>(node_, topic, codec, rtsp_port);
    streamers_[stream_key] = streamer;

    return streamer;
}

void GstRTSPStreamerManager::removeStreamer(const std::string & topic)
{
    std::lock_guard<std::mutex> lock(streamers_mutex_);
    auto it = streamers_.begin();
    while (it != streamers_.end())
    {
        if (it->first.find(topic + "_") == 0)
        {
            it->second->stop();
            it = streamers_.erase(it);
        } else
        {
            ++it;
        }
    }
}

std::shared_ptr<GstRTSPStreamer> GstRTSPStreamerManager::getStreamer(const std::string & topic)
{
    std::lock_guard<std::mutex> lock(streamers_mutex_);
    for (const auto & pair : streamers_)
    {
        if (pair.first.find(topic + "_") == 0)
        {
            return pair.second;
        }
    }
    return nullptr;
}

std::vector<std::string> GstRTSPStreamerManager::getActiveStreams() const
{
    std::lock_guard<std::mutex> lock(streamers_mutex_);
    std::vector<std::string> active_streams;
    for (const auto & pair : streamers_)
    {
        if (pair.second->isActive())
        {
            active_streams.push_back(pair.first);
        }
    }
    return active_streams;
}

void GstRTSPStreamerManager::cleanup()
{
    std::lock_guard<std::mutex> lock(streamers_mutex_);
    for (auto & pair : streamers_)
    {
        pair.second->stop();
    }
    streamers_.clear();
}

void GstRTSPStreamerManager::cleanupInactiveStreamers()
{
    std::lock_guard<std::mutex> lock(streamers_mutex_);
    auto it = streamers_.begin();
    while (it != streamers_.end())
    {
        if (!it->second->isActive())
        {
            it = streamers_.erase(it);
        } else
        {
            ++it;
        }
    }
}

}  // namespace web_video_server
