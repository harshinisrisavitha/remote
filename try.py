# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import Image,CameraInfo
# from cv_bridge import CvBridge
# import cv2
# import cv2.aruco as aruco
# import numpy as np

# from message_filters import ApproximateTimeSynchronizer, Subscriber

# class ArucoPose(Node):
#     def __init__(self):
#         super().__init__('aruco_pose')
#         self.bridge=CvBridge()
#         self.camera_matrix=None
#         self.dist_coeffs=None
#         self.marker_len=0.15

#         self.img_sub=Subscriber(self,Image,'/front_cam/zed_node/rgb/image_rect_color')
#         self.info_sub=Subscriber(self,CameraInfo,'/front_cam/zed_node/rgb/camera_info')

#         self.ts=ApproximateTimeSynchronizer([self.img_sub,self.info_sub],queue_size=0.1,slop=0.1)
#         self.ts.registerCallback(self.callback)

#         self.aruco_dict=aruco.getPredefinedDictionary(aruco.DICT_4X4_100)
#         self.parameters=aruco.DetectorParameters_create()

#     def callback(self,img_msg,info_msg):
#         frame=self.bridge.imgmsg_to_cv2(img_msg,desired_encoding='bgr8')


#         if self.camera_matrix is None:
#             self.camera_matrix=np.array(info_msg.k).reshape(3,3)
#             self.dist_coeffs=np.array(info_msg.d)

#         gray=cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)

#         corners,ids,_=aruco.detectMarkers(gray,self.aruco_dict,parameters=self.parameters)

#         if ids is not None:
#             for i in range(len(ids)):
#                 pts = corners[i][0].astype(int)
#                 cv2.polylines(frame, [pts], True, (0, 255, 0), 2)

#                 cx=int(pts[:,0].mean())
#                 cy=int(pts[:,1].mean())

#                 rvec, tvec, _ = aruco.estimatePoseSingleMarkers([corners[i]], self.marker_len, self.camera_matrix, self.dist_coeffs)
#                 cv2.drawFrameAxes(frame, self.camera_matrix, self.dist_coeffs, rvec[0], tvec[0], 0.03)

#                 self.get_logger().info(f"ID {ids[i][0]} Pose -> R: {rvec[0]} | T: {tvec[0]}")
        
#         cv2.imshow("Aruco Detection", frame)
#         cv2.waitKey(1)

# def main(args=None):
#     rclpy.init(args=args)
#     node = ArucoPose()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()
#     cv2.destroyAllWindows()


# if __name__ == '__main__':
#     main()


import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
import cv2.aruco as aruco

class ArucoDetector(Node):
    def __init__(self):
        super().__init__('aruco_detector_node')

        # Parameters
        self.marker_length = 0.15  # in meters (adjust based on your marker)

        # CvBridge
        self.bridge = CvBridge()

        # Image Subscriber
        self.image_sub = self.create_subscription(
            Image,
            '/front_cam/zed_node/rgb/image_rect_color',
            self.image_callback,
            10)

        # Camera Info Subscriber
        self.cam_info_sub = self.create_subscription(
            CameraInfo,
            '/front_cam/zed_node/rgb/camera_info',
            self.camera_info_callback,
            10)

        # Image Publisher
        self.image_pub = self.create_publisher(
            Image,
            '/aruco/image_marked',
            10)

        # ArUco dictionary & params
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_100)
        self.aruco_params = aruco.DetectorParameters_create()

        # Camera calibration placeholders
        self.camera_matrix = None
        self.dist_coeffs = None

    def camera_info_callback(self, msg):
        try:
            self.camera_matrix = np.array(msg.k, dtype=np.float32).reshape((3, 3))
            self.dist_coeffs = np.array(msg.d, dtype=np.float32)
            self.get_logger().info("Camera calibration loaded.")
  # Important: stop after receiving once
        except Exception as e:
            self.get_logger().error(f"Failed to parse camera info: {e}")
            self.cam_info_sub.destroy()


    def image_callback(self, msg):
        if self.camera_matrix is None or self.dist_coeffs is None:
            self.get_logger().warn('Waiting for camera intrinsics...')
            return

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        # rest of ArUco detection logic here...
        except Exception as e:
            self.get_logger().error(f"Image callback error: {e}")


        # Convert ROS Image to OpenCV
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        corners, ids, rejected = aruco.detectMarkers(
            gray, self.aruco_dict, parameters=self.aruco_params)

        if ids is not None:
            for i in range(len(ids)):
                pts = corners[i][0].astype(int)
                cv2.polylines(frame, [pts], isClosed=True, color=(0, 255, 0), thickness=2)

                center_x = int(pts[:, 0].mean())
                center_y = int(pts[:, 1].mean())
                cv2.putText(frame, f"ID:{ids[i][0]}", (center_x - 20, center_y - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

                rvec, tvec, _ = aruco.estimatePoseSingleMarkers(
                    [corners[i]], self.marker_length, self.camera_matrix, self.dist_coeffs)

                # aruco.drawFrameAxes(frame, self.camera_matrix, self.dist_coeffs, rvec, tvec, 0.1)


                self.get_logger().info(f"ID {ids[i][0]} Pose:\n"
                                       f"Translation: {tvec[0]}\n"
                                       f"Rotation: {rvec[0]}")

        # Publish annotated image
        marked_img_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        self.image_pub.publish(marked_img_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
