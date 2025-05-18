# aruco_markers
Dictionaries:
  DICT_4X4_50
  DICT_4X4_100
  DICT_4X4_250
  DICT_4X4_1000
  DICT_5X5_50
  DICT_5X5_100
  DICT_5X5_250
  DICT_5X5_1000
  DICT_6X6_50
  DICT_6X6_100
  DICT_6X6_250
  DICT_6X6_1000
  DICT_7X7_50
  DICT_7X7_100
  DICT_7X7_250
  DICT_7X7_1000
  DICT_ARUCO_ORIGINAL
  DICT_APRILTAG_16h5
  DICT_APRILTAG_25h9
  DICT_APRILTAG_36h10
  DICT_APRILTAG_36h11 

  ```sh
  source install/setup.bash
  ros2 run aruco_markers aruco_markers --ros-args \
    -p marker_size:=0.1 \
    -p camera_frame:=front_cam_center_optical_frame
    -p image_topic:=/front_cam/zed_node/rgb/image_rect_color
    -p camera_info_topic:=/front_cam/zed_node/rgb/camera_info
    -p dictionary:=DICT_4X4_100
