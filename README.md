# web_video_server - HTTP Streaming of ROS Image Topics in Multiple Formats

This node provides HTTP streaming of ROS image topics in various formats, making it easy to view robot camera feeds and other image topics in a web browser without requiring special plugins or extensions.

## Features

- Stream ROS image topics over HTTP in multiple formats:
  - MJPEG (Motion JPEG)
  - VP8 (WebM)
  - VP9 (WebM)
  - H264 (MP4)
  - PNG streams
  - ROS compressed image topics
- **NEW: RTSP streaming support** for low-latency video streaming
- Adjustable quality, size, and other streaming parameters
- Web interface to browse available image topics
- Single image snapshot capability
- Support for different QoS profiles in ROS 2
- Support for PointCloud2 topics (converted to depth images)

## Installation

### Dependencies

- ROS (Noetic) or ROS 2 (Humble+)
- OpenCV
- FFmpeg/libav
- Boost
- async_web_server_cpp
 
### Installing packages

For newer ROS2 distributions (humble, jazzy, rolling) it is possible to install web_video_server as a package:

```
sudo apt install ros-${ROS_DISTRO}-web-video-server
```

### Building from Source

Create a ROS workspace if you don't have one:
```bash
mkdir -p ~/ros_ws/src
cd ~/ros_ws/src
```

Clone this repository:
```bash
# ROS 2
git clone https://github.com/RobotWebTools/web_video_server.git
# ROS 1
git clone https://github.com/RobotWebTools/web_video_server.git -b ros1
```

Install dependencies with rosdep:
```bash
cd ~/ros_ws
rosdep update
rosdep install --from-paths src -i
```

Build the package and source your workspace:
```bash
colcon build --packages-select web_video_server
source install/setup.bash
```

## Usage

### Starting the Server

#### Basic Launch (HTTP + RTSP enabled)

```bash
# ROS 1
rosrun web_video_server web_video_server

# ROS 2 - Basic launch with both HTTP and RTSP enabled
ros2 run web_video_server web_video_server --ros-args -p rtsp_server_enabled:=true -p http_server_enabled:=true
```

#### Launch Options

**HTTP Only (default behavior):**
```bash
ros2 run web_video_server web_video_server
```

**RTSP Only:**
```bash
ros2 run web_video_server web_video_server --ros-args -p rtsp_server_enabled:=true -p http_server_enabled:=false
```

**Both HTTP + RTSP (recommended for testing):**
```bash
ros2 run web_video_server web_video_server --ros-args -p rtsp_server_enabled:=true -p http_server_enabled:=true
```

**With custom ports:**
```bash
ros2 run web_video_server web_video_server --ros-args -p rtsp_server_enabled:=true -p http_server_enabled:=true -p port:=8080 -p rtsp_port:=8554
```


### Configuration

#### Server Configuration Parameters

| Parameter | Type | Default | Possible Values | Description |
|-----------|------|---------|----------------|-------------|
| `port` | int | 8080 | Any valid port number | HTTP server port |
| `address` | string | "0.0.0.0" | Any valid IP address | HTTP server address (0.0.0.0 allows external connections) |
| `server_threads` | int | 1 | 1+ | Number of server threads for handling HTTP requests |
| `ros_threads` | int | 2 | 1+ | Number of threads for ROS message handling |
| `verbose` | bool | false | true, false | Enable verbose logging |
| `default_stream_type` | string | "mjpeg" | "mjpeg", "vp8", "vp9", "h264", "png", "ros_compressed" | Default format for video streams |
| `publish_rate` | double | -1.0 | -1.0 or positive value | Rate for republishing images (-1.0 means no republishing) |
| `http_enabled` | bool | true | true, false | Enable HTTP streaming server (at least one of http_enabled or rtsp_enabled must be true) |
| `rtsp_enabled` | bool | true | true, false | Enable RTSP streaming functionality |
| `rtsp_port` | int | 8554 | Any valid port number | RTSP server port |
| `rtsp_address` | string | "0.0.0.0" | Any valid IP address | RTSP server address |

#### Running with Custom Parameters

You can configure the server by passing parameters via the command line:

```bash
# ROS 1
rosrun web_video_server web_video_server _port:=8081 _address:=localhost _server_threads:=4

# ROS 2
ros2 run web_video_server web_video_server --ros-args -p port:=8081 -p address:=localhost -p server_threads:=4
```

### View Available Streams
```
http://localhost:8080/
```
The interface allows quick navigation between different topics and formats without having to manually construct URLs.

This page displays:
- All available ROS image topics currently being published
- Direct links to view each topic in different formats:
  - Web page with streaming image
  - Direct stream
  - Single image snapshot

### Stream an Image Topic

There are two ways to stream the Image, as a HTML page via 
```
http://localhost:8080/stream_viewer?topic=/camera/image_raw
```
or as a HTTP multipart stream on

```
http://localhost:8080/stream?topic=/camera/image_raw
```
#### URL Parameters for Streaming

The following parameters can be added to the stream URL:

| Parameter | Type | Default | Possible Values | Description |
|-----------|------|---------|----------------|-------------|
| `topic` | string | (required) | Any valid ROS image topic | The ROS image topic to stream |
| `type` | string | "mjpeg" | "mjpeg", "vp8", "vp9", "h264", "png", "ros_compressed" | Stream format |
| `width` | int | 0 | 0+ | Width of output stream (0 = original width) |
| `height` | int | 0 | 0+ | Height of output stream (0 = original height) |
| `quality` | int | 95 | 1-100 | Quality for MJPEG and PNG streams |
| `bitrate` | int | 100000 | Positive integer | Bitrate for H264/VP8/VP9 streams in bits/second |
| `invert` | flag | not present | present/not present | Invert image when parameter is present |
| `default_transport` | string | "raw" | "raw", "compressed", "theora" | Image transport to use |
| `qos_profile` | string | "default" | "default", "system_default", "sensor_data", "services_default" | QoS profile for ROS 2 subscribers |

Examples:

```
# Stream an MJPEG at 640x480 with 90% quality
http://localhost:8080/stream?topic=/camera/image_raw&type=mjpeg&width=640&height=480&quality=90

# Stream H264 with higher bitrate
http://localhost:8080/stream?topic=/camera/image_raw&type=h264&bitrate=500000

# Stream with inverted image (rotated 180°)
http://localhost:8080/stream?topic=/camera/image_raw&invert

```

### Get a Snapshot
It is also possible to get a single image snapshot 
```
http://localhost:8080/snapshot?topic=/camera/image_raw
```
#### URL Parameters for Snapshot

| Parameter | Type | Default | Possible Values | Description |
|-----------|------|---------|----------------|-------------|
| `topic` | string | (required) | Any valid ROS image topic | The ROS image topic to stream |
| `width` | int | 0 | 0+ | Width of output picture (0 = original width) |
| `height` | int | 0 | 0+ | Height of output picture (0 = original height) |
| `quality` | int | 95 | 1-100 | Quality for JPEG snapshots |
| `invert` | flag | not present | present/not present | Invert image when parameter is present |
| `default_transport` | string | "raw" | "raw", "compressed", "theora" | Image transport to use |
| `qos_profile` | string | "default" | "default", "system_default", "sensor_data", "services_default" | QoS profile for ROS 2 subscribers |

### RTSP Streaming

The web_video_server now supports RTSP streaming for low-latency video streaming. This is particularly useful for applications requiring real-time video feeds.

#### Enable RTSP Streaming

To create an RTSP stream, make an HTTP request to:
```
http://localhost:8080/rtsp_stream?topic=/camera/image_raw
```

Using curl:
```bash
curl "http://localhost:8080/rtsp_stream?topic=/image_raw&bitrate=1000000"
```

This will return a JSON response with the RTSP URL:
```json
{"rtsp_url": "rtsp://localhost:8554/stream?topic=/image_raw&type=h264"}
```

#### RTSP URL Parameters

| Parameter | Type | Default | Possible Values | Description |
|-----------|------|---------|----------------|-------------|
| `topic` | string | (required) | Any valid ROS image topic | The ROS image topic to stream |
| `codec` | string | "h264" | "h264", "libx264" | Video codec to use for encoding |

#### Using RTSP Streams

Once you have the RTSP URL, you can view the stream using:

**GStreamer (Recommended):**
```bash
# Full pipeline with RTP depayloading
gst-launch-1.0 rtspsrc location=rtsp://localhost:8554/stream?topic=/image_raw\&type=h264 ! rtph264depay ! h264parse ! avdec_h264 ! videoconvert ! autovideosink

# Simple pipeline
gst-launch-1.0 rtspsrc location=rtsp://localhost:8554/stream?topic=/image_raw\&type=h264 ! decodebin ! videoconvert ! autovideosink
```

**FFplay:**
```bash
# Basic playback
ffplay rtsp://localhost:8554/stream?topic=/image_raw\&type=h264

# Low latency playback (recommended)
ffplay -fflags nobuffer -flags low_delay rtsp://localhost:8554/stream?topic=/image_raw\&type=h264

# With TCP transport (more reliable)
ffplay -rtsp_transport tcp rtsp://localhost:8554/stream?topic=/image_raw\&type=h264
```

**VLC Media Player:**
```bash
vlc rtsp://localhost:8554/stream?topic=/image_raw\&type=h264
```

**OpenCV Python:**
```python
import cv2

cap = cv2.VideoCapture('rtsp://localhost:8554/stream?topic=/image_raw&type=h264')
while True:
    ret, frame = cap.read()
    if ret:
        cv2.imshow('RTSP Stream', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    else:
        break

cap.release()
cv2.destroyAllWindows()
```

#### RTSP Configuration

You can configure RTSP streaming with additional parameters:

```bash
# Enable RTSP with custom port
ros2 run web_video_server web_video_server --ros-args -p rtsp_enabled:=true -p rtsp_port:=8555

# Disable RTSP streaming
ros2 run web_video_server web_video_server --ros-args -p rtsp_enabled:=false

# Run RTSP-only mode (disable HTTP server)
ros2 run web_video_server web_video_server --ros-args -p http_enabled:=false

# Run with both HTTP and RTSP disabled (this will cause an error)
# ros2 run web_video_server web_video_server --ros-args -p http_enabled:=false -p rtsp_enabled:=false
```

#### Server Mode Configuration

The web_video_server supports flexible server configuration:

- **HTTP + RTSP Mode (Default)**: Both HTTP and RTSP streaming are enabled
- **HTTP Only Mode**: Disable RTSP with `-p rtsp_enabled:=false`
- **RTSP Only Mode**: Disable HTTP with `-p http_enabled:=false`

**Important**: At least one of `http_enabled` or `rtsp_enabled` must be true. The server will refuse to start if both are disabled.

In RTSP-only mode, the HTTP server is completely disabled to save resources, but you can still create RTSP streams programmatically using the RTSP streamer manager.

## About

This project is released as part of the [Robot Web Tools](https://robotwebtools.github.io/) effort.

## License
web_video_server is released with a BSD license. For full terms and conditions, see the [LICENSE](LICENSE) file.

## Authors
See the [AUTHORS](AUTHORS.md) file for a full list of contributors.
