import rclpy
import cv2
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Header
import tf_transformations
import tf2_ros
import math


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
        self.declare_parameter("map.topic")
        self.declare_parameter("update_map.length_x")
        self.declare_parameter("update_map.length_y")
        self.declare_parameter("update_map.resolution")
        self.declare_parameter("update_map.robot_pose.x")
        self.declare_parameter("update_map.robot_pose.y")
        self.declare_parameter("update_map.robot_pose.theta")
        self.declare_parameter("calibratioself.src")
        self.declare_parameter("calibratioself.marker_size")
        self.declare_parameter("calibratioself.marker_pose.x")
        self.declare_parameter("calibratioself.marker_pose.y")
        self.declare_parameter("calibration.marker_pose.theta")

        # Load Parameters
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
        self.M = np.array(self.get_parameter("camera.M").value).reshape(3, 3)

        self.bridge = CvBridge()

        # Setup Publishers
        self.map_publisher = self.create_publisher(
            OccupancyGrid, self.get_parameter("map.topic").value, 10
        )

        map_x = self.get_parameter("update_map.length_x").value
        map_y = self.get_parameter("update_map.length_y").value
        map_resolution = self.get_parameter("update_map.resolution").value
        self.map_msg = OccupancyGrid()
        self.map_msg.header = Header()
        self.map_msg.header.frame_id = 'base_footprint'
        self.map_msg.info.resolution = map_resolution * 2
        self.map_msg.info.height = int(map_y / map_resolution / 2)
        self.map_msg.info.width = int(map_x / map_resolution / 2)

        self.map_msg.info.origin.position.x = 0.0
        self.map_msg.info.origin.position.y = 0.5

        q = tf_transformations.quaternion_from_euler(
            0, 0, -math.pi / 2)
        self.map_msg.info.origin.orientation.x = q[0]
        self.map_msg.info.origin.orientation.y = q[1]
        self.map_msg.info.origin.orientation.z = q[2]
        self.map_msg.info.origin.orientation.w = q[3]

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
        map_x = self.get_parameter("update_map.length_x").value
        map_y = self.get_parameter("update_map.length_y").value
        map_resolution = self.get_parameter("update_map.resolution").value
        size = (int(map_y / map_resolution), int(map_x / map_resolution))

        ret, frame = cv2.threshold(frame, 200, 100, cv2.THRESH_BINARY_INV)
        frame += 1
        map_img = cv2.warpPerspective(
            frame, self.M, size, flags=cv2.INTER_NEAREST, borderMode=cv2.BORDER_CONSTANT, borderValue=0)

        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        map_img = cv2.morphologyEx(
            map_img, cv2.MORPH_OPEN, kernel, iterations=2)

        map_img = map_img.astype(np.int8)
        map_img -= 1

        map_img = cv2.resize(map_img, (500, 500), interpolation=cv2.INTER_NEAREST)
        self.map_msg.data = np.fliplr(map_img).ravel().tolist()[::-1]
        self.map_publisher.publish(self.map_msg)
        pass


def main(args=None):
    rclpy.init(args=args)
    andante_camera_linetrace = AndanteCameraLinetrace()
    rclpy.spin(andante_camera_linetrace)
    andante_camera_linetrace.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
