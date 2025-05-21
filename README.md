# Aruco Markers Package

## How to Launch

1. **Launch the Aruco marker detection node:**

   ```sh
   ros2 launch aruco_markers aruco_launch.py

3. **Open a new terminal and run the image viewer to see the camera feed with detected markers:**

   ```sh
   ros2 run rqt_image_view rqt_image_view

4. **To get the pose data of the detected markers, echo the /aruco/markers topic:**
   ```sh
   ros2 topic echo /aruco/markers

