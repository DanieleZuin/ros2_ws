# ROS 2 Hand-Gesture Teleoperation System

An end-to-end robotics project demonstrating a distributed architecture for real-time robot control using Computer Vision and AI.

## 🚀 Overview
This project enables a user to teleoperate a robot (simulated via `turtlesim`) using hand gestures captured by a webcam. It leverages a multi-node ROS 2 architecture to decouple perception from actuation.

### Key Features
- **AI-Powered Perception**: Uses Google MediaPipe for high-performance hand landmark detection.
- **Distributed "Plan C" Architecture**: A Python-based Flask server runs natively on the host (macOS) to bypass VM USB latency, streaming video to the ROS 2 node via HTTP.
- **Custom Kinematic Mapping**: Translates hand openness to linear velocity (throttle) and wrist tilt to angular velocity (steering).
- **Visualization**: Integrated OpenCV debug overlay and RViz2 support.

## 🏗️ Architecture
The system is orchestrated through the following components:
1. **Host Streamer (Mac)**: Captures FaceTime/USB camera frames and serves an MJPEG stream.
2. **Vision Node (Ubuntu VM)**: 
   - Subscribes to the video stream.
   - Processes landmarks to calculate `Twist` messages.
   - Publishes to the `/cmd_vel` topic.
3. **Robot Node (Turtlesim)**: Executes the movement commands in the simulation environment.

## 🛠️ Tech Stack
- **Middleware**: ROS 2 Humble
- **Libraries**: OpenCV, MediaPipe, NumPy, Flask
- **Language**: Python 3
- **Environment**: UTM Virtualization (Ubuntu 22.04 on Apple Silicon)

## 🔧 Installation & Usage

