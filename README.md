# Aruco Marker Detection using ROS 2 & OpenCV

- Subscribes to rectified color image and camera info topics
- Detects ArUco markers in real-time using OpenCV
- Estimates the pose (rotation + translation) of each marker
- Publishes the annotated image with axes and marker IDs

| Topic | Description |
|-------|-------------|
| `/front_cam/zed_node/rgb/image_rect_color` | Input image topic |
| `/front_cam/zed_node/rgb/camera_info` | Intrinsic parameters |
| `/aruco_detections/image` | Annotated image (published) |
| `/aruco_detections/poses` | Marker ID + pose (published) |

dictionary = cv2.aruco.DICT_4X4_50

## **To run this package:**

 	``sh
  ros2 run sruco_detect detect
