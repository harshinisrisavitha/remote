# Aruco Markers Package

## How to Launch

1. **Launch the Aruco marker detection node:**
2. **Open a new terminal and run the image viewer to see the camera feed with detected markers:**
3. **To get the pose data of the detected markers, echo the /aruco/markers topic:**

```sh
ros2 launch aruco_markers aruco_launch.py
ros2 run rqt_image_view rqt_image_view
ros2 topic echo /aruco/markers
