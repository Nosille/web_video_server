# RTSP Streaming Documentation

## Overview
The web_video_server has been updated to provide RTSP streaming functionality that mirrors the HTTP streaming API. The RTSP functionality follows the same URL pattern and query parameter structure as the HTTP streaming.

## Changes Made

### 1. Updated HTTP API for RTSP Stream Creation
**OLD:** `curl "http://localhost:8080/rtsp_stream?topic=/image_raw&codec=h264"`
**NEW:** `curl "http://localhost:8080/rtsp_stream?topic=/image_raw&type=h264"`

### 2. Updated RTSP URL Format
**OLD:** `rtsp://localhost:8554//image_raw`
**NEW:** `rtsp://localhost:8554/stream?topic=/image_raw&type=h264`

### 3. Stream Type to Codec Mapping
The system now maps HTTP stream types to appropriate RTSP codecs:
- `type=h264` → `codec=libx264`
- `type=vp8` → `codec=libvpx`
- `type=vp9` → `codec=libvpx-vp9`
- Other types → `codec=libx264` (default)

### 4. Improved RTSP Protocol Support
- Added proper OPTIONS method handling
- Added proper CSeq header handling in all RTSP responses
- Fixed RTSP method signatures to include request context

## Usage

### Starting the Server
```bash
# Start with both HTTP and RTSP enabled
ros2 run web_video_server web_video_server --ros-args -p http_enabled:=true -p rtsp_enabled:=true

# Or start with only RTSP
ros2 run web_video_server web_video_server --ros-args -p http_enabled:=false -p rtsp_enabled:=true
```

### Creating RTSP Streams
```bash
# Create H.264 stream
curl "http://localhost:8080/rtsp_stream?topic=/image_raw&type=h264"

# Create VP8 stream
curl "http://localhost:8080/rtsp_stream?topic=/image_raw&type=vp8"

# Create VP9 stream
curl "http://localhost:8080/rtsp_stream?topic=/image_raw&type=vp9"
```

### Using with GStreamer
```bash
# Basic H.264 streaming
gst-launch-1.0 rtspsrc location="rtsp://localhost:8554/stream?topic=/image_raw&type=h264" ! rtph264depay ! h264parse ! avdec_h264 ! videoconvert ! autovideosink

# Auto-detection
gst-launch-1.0 rtspsrc location="rtsp://localhost:8554/stream?topic=/image_raw&type=h264" ! decodebin ! videoconvert ! autovideosink

# Save to file
gst-launch-1.0 rtspsrc location="rtsp://localhost:8554/stream?topic=/image_raw&type=h264" ! rtph264depay ! h264parse ! mp4mux ! filesink location=output.mp4

# Stream over network
gst-launch-1.0 rtspsrc location="rtsp://localhost:8554/stream?topic=/image_raw&type=h264" ! rtph264depay ! h264parse ! rtph264pay ! udpsink host=192.168.1.100 port=5000
```

## Comparison with HTTP Streaming

### HTTP Streaming
- URL: `http://localhost:8080/stream?topic=/image_raw&type=mjpeg`
- Direct browser access
- MJPEG, PNG, H.264, VP8, VP9 support
- Real-time streaming

### RTSP Streaming
- URL: `rtsp://localhost:8554/stream?topic=/image_raw&type=h264`
- Requires RTSP client (GStreamer, VLC, etc.)
- H.264, VP8, VP9 support (transcoded from ROS topics)
- Real-time streaming with RTP protocol

## Testing

### Test with USB Camera
```bash
# Terminal 1: Start USB camera
ros2 run usb_cam usb_cam_node_exe --ros-args -p video_device:="/dev/video0"

# Terminal 2: Start web_video_server
ros2 run web_video_server web_video_server --ros-args -p http_enabled:=true -p rtsp_enabled:=true

# Terminal 3: Create RTSP stream
curl "http://localhost:8080/rtsp_stream?topic=/image_raw&type=h264"

# Terminal 4: View stream
gst-launch-1.0 rtspsrc location="rtsp://localhost:8554/stream?topic=/image_raw&type=h264" ! decodebin ! videoconvert ! autovideosink
```

### Test with VLC
```bash
vlc rtsp://localhost:8554/stream?topic=/image_raw&type=h264
```

## Current Limitations

1. **RTSP Protocol Implementation**: The current implementation has a simplified RTSP protocol that may not work with all RTSP clients. GStreamer connection gets stuck during the "Retrieving server options" phase.

2. **RTP Packetization**: The RTP packets are simplified and may not follow the complete RTP specification.

3. **SDP Description**: The SDP description is basic and may need enhancements for full compatibility.

## Future Improvements

1. **Complete RTSP Protocol Support**: Implement full RTSP 1.0 specification
2. **Proper RTP Packetization**: Implement RFC-compliant RTP packet structure
3. **Multiple Stream Support**: Support multiple concurrent streams on different ports
4. **Authentication**: Add RTSP authentication support
5. **Stream Discovery**: Automatic stream discovery for available topics

## Architecture

The RTSP streaming functionality is built on top of the existing HTTP streaming architecture:

1. **RTSPStreamerManager**: Manages multiple RTSP streams
2. **RTSPStreamer**: Individual stream handler for each topic
3. **HTTP Endpoint**: `/rtsp_stream` endpoint for creating streams
4. **RTSP Protocol**: Custom RTSP server implementation
5. **RTP Streaming**: UDP-based RTP packet transmission

This design ensures consistency with the existing HTTP streaming while providing the benefits of RTSP protocol for real-time video streaming.
