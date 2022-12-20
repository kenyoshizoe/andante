import rclpy
import cv2
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError


class AndanteCameraLinetrace(Node):
    def __init__(self):
        super().__init__("andante_camera_linetrace")
        self.declare_parameter("camera.use_topic")
        self.declare_parameter("camera.img_topic")
        self.declare_parameter("camera.camera_info_topic")
        self.declare_parameter("camera.camera_src")
        self.declare_parameter("camera.width")
        self.declare_parameter("camera.height")
        self.declare_parameter("camera.fps")
        self.declare_parameter("camera.mode")
        self.declare_parameter("camera.camera_matrix.fx")
        self.declare_parameter("camera.camera_matrix.fy")
        self.declare_parameter("camera.camera_matrix.cx")
        self.declare_parameter("camera.camera_matrix.cy")
        self.declare_parameter("camera.dist")
        self.declare_parameter("camera.M")
        self.declare_parameter("map.length_x")
        self.declare_parameter("map.length_y")
        self.declare_parameter("map.resolution")
        self.declare_parameter("map.robot_pose.x")
        self.declare_parameter("map.robot_pose.y")
        self.declare_parameter("map.robot_pose.theta")
        self.declare_parameter("calibratioself.src")
        self.declare_parameter("calibratioself.marker_size")
        self.declare_parameter("calibratioself.marker_pose.x")
        self.declare_parameter("calibratioself.marker_pose.y")
        self.declare_parameter("calibration.marker_pose.theta")

        self.camera_matrix = None
        self.distcoeffs = None

        if self.get_parameter("camera.use_topic"):
            img_topic = self.get_parameter("camera.img_topic").value
            camera_info_topic = self.get_parameter(
                "camera.camera_info_topic").value
            self.img_sub = self.create_subscription(
                Image, img_topic,
                self.img_callback, 10
            )
            self.camera_info_sub = self.create_subscription(
                CameraInfo, camera_info_topic,
                self.camera_info_callback, 10
            )
        else:
            fx = self.get_parameter("camera.camera_matrix.fx").value
            fy = self.get_parameter("camera.camera_matrix.fy").value
            cx = self.get_parameter("camera.camera_matrix.cx").value
            cy = self.get_parameter("camera.camera_matrix.cy").value
            self.camera_matrix = np.array(
                [[fx, 0, cx],
                 [0, fy, cy],
                 [0, 0, 1]
                 ])
            self.distcoeffs = np.array(
                self.get_parameter("camera.dist").value
            ).reshape(-1)

        # 2値画像を収縮する。
        self.bridge = CvBridge()

        self.M = np.array(self.get_parameter("camera.M").value).reshape(3, 3)

    def img_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "mono8")
        except CvBridgeError as e:
            print(e)
            return
        self.process_img(frame)

    def camera_info_callback(self, msg):
        self.camera_matrix = np.array(msg.k).reshape(3, 3)
        self.distcoeffs = np.array(msg.d).reshape(-1)

    def img_reader(self):
        pass

    def process_img(self, frame):
        map_x = self.get_parameter("map.length_x").value
        map_y = self.get_parameter("map.length_y").value
        map_resolution = self.get_parameter("map.resolution").value
        size = (int(map_y / map_resolution), int(map_x / map_resolution))
        map_img = cv2.warpPerspective(frame, self.M, size)
        ret, map_img = cv2.threshold(map_img, 200, 255, cv2.THRESH_BINARY)

        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        map_img = cv2.morphologyEx(
            map_img, cv2.MORPH_OPEN, kernel, iterations=2)

        cv2.imshow('test', map_img)
        cv2.waitKey(1)
        pass


def main(args=None):
    rclpy.init(args=args)
    andante_camera_linetrace = AndanteCameraLinetrace()
    rclpy.spin(andante_camera_linetrace)
    andante_camera_linetrace.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
