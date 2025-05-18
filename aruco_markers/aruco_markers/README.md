# aruco_markers
'''sh
source install/setup.bash
ros2 run aruco_markers aruco_markers --ros-args \
  -p marker_size:=0.1 \
  -p camera_frame:=front_cam_center_optical_frame
  -p image_topic:=/front_cam/zed_node/rgb/image_rect_color
  -p camera_info_topic:=/front_cam/zed_node/rgb/camera_info
  -p dictionary:=DICT_4X4_100
