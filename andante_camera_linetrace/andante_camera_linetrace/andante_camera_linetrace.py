import rclpy
import cv2
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
from nav_msgs.msg import OccupancyGrid, Odometry
from std_msgs.msg import Header
import tf_transformations
import tf2_ros
import math


class AndanteCameraLinetrace(Node):
    def __init__(self):
        super().__init__("andante_camera_linetrace")
        self.declare_params()
        self.param_loaded = False

        self.M = np.array(self.get_parameter("camera.M").value).reshape(3, 3)
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
            self.bridge = CvBridge()
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
            ).reshape(-1)-1
            self.param_loaded = True
            self.cap = cv2.VideoCapture(
                self.get_parameter("camera.camera_src").value
            )

        map_size = self.get_parameter("map.size").value
        map_resolution = self.get_parameter("map.resolution").value
        self.map_img = np.full(
            (int(map_size / map_resolution), int(map_size / map_resolution)), 100, dtype=np.int8)
        self.map_pub = self.create_publisher(
            OccupancyGrid, '/map', 10
        )

        self.create_subscription(
            Odometry, self.get_parameter(
                "odom_topic").value, self.map_update, 10
        )
        self.pre_pose = None

    def declare_params(self):
        self.declare_parameter("base_frame")
        self.declare_parameter("odom_topic")
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
        self.declare_parameter("map.size")
        self.declare_parameter("map.resolution")
        self.declare_parameter("calibration.src")
        self.declare_parameter("calibration.marker_size")
        self.declare_parameter("calibration.marker_pose.x")
        self.declare_parameter("calibration.marker_pose.y")
        self.declare_parameter("calibration.marker_pose.theta")

    def img_callback(self, msg):
        if self.param_loaded == False:
            return
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "mono8")
        except CvBridgeError as e:
            print(e)
            return
        self.process_img(frame)

    def camera_info_callback(self, msg):
        if self.param_loaded == False:
            self.camera_matrix = np.array(msg.k).reshape(3, 3)
            self.distcoeffs = np.array(msg.d).reshape(-1)
            self.param_loaded = True

    def img_reader(self):
        pass

    def process_img(self, frame):
        map_size = self.get_parameter("map.size").value
        map_resolution = self.get_parameter("map.resolution").value

        ret, frame = cv2.threshold(frame, 200, 100, cv2.THRESH_BINARY_INV)
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        frame = cv2.morphologyEx(
            frame, cv2.MORPH_CLOSE, kernel, iterations=2)

        frame += 1
        observe = cv2.warpPerspective(
            frame, self.M, self.map_img.shape, borderMode=cv2.BORDER_CONSTANT, borderValue=-1).astype(np.int8)
        observe -= 1

        self.map_img = np.where(observe == -1, self.map_img, observe)

        # publish map_msg
        map_msg = OccupancyGrid()
        map_msg.header = Header()
        map_msg.header.frame_id = self.get_parameter("base_frame").value
        map_msg.info.resolution = map_resolution
        map_msg.info.width = int(map_size / map_resolution)
        map_msg.info.height = int(map_size / map_resolution)
        map_msg.info.origin.position.x = - map_size / 2
        map_msg.info.origin.position.y = - map_size / 2
        map_data = cv2.flip(self.map_img, 0).flatten().tolist()
        map_msg.data = map_data
        self.map_pub.publish(map_msg)

    def map_update(self, msg):
        if self.pre_pose is None:
            self.pre_pose = msg.pose.pose

        # transform map
        dx = msg.pose.pose.position.x - self.pre_pose.position.x
        dy = msg.pose.pose.position.y - self.pre_pose.position.y
        pre_theta = tf_transformations.euler_from_quaternion([
            msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
        )[2]
        cur_theta = tf_transformations.euler_from_quaternion([
            self.pre_pose.orientation.x, self.pre_pose.orientation.y,
            self.pre_pose.orientation.z, self.pre_pose.orientation.w
        ])[2]
        dtheta = -(cur_theta - pre_theta)

        if abs(math.sqrt(dx**2 + dy**2)) < 0.01 and abs(dtheta) < (math.pi / 36):
            return

        map_resolution = self.get_parameter("map.resolution").value
        dx_img = (math.cos(cur_theta) * dx +
                  math.sin(cur_theta) * dy) / map_resolution
        dy_img = (-math.sin(cur_theta) * dx +
                  math.cos(cur_theta) * dy) / map_resolution
        M_trans = np.array([
            [1, 0, -dx_img],
            [0, 1, -dy_img]
        ]).astype(np.float32)
        M_rotate = cv2.getRotationMatrix2D(
            (self.map_img.shape[1] / 2, self.map_img.shape[0]/2), -dtheta/math.pi*180, 1.0)

        self.map_img = self.map_img.astype(np.float32)
        self.map_img = cv2.warpAffine(
            self.map_img, M_trans, self.map_img.shape, flags=cv2.INTER_NEAREST, borderValue=100)
        self.map_img = cv2.warpAffine(
            self.map_img, M_rotate, self.map_img.shape, flags=cv2.INTER_NEAREST, borderValue=100)
        # remove noise
        self.map_img = self.map_img.astype(np.uint8)
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
        self.map_img = cv2.morphologyEx(
            self.map_img, cv2.MORPH_CLOSE, kernel, iterations=1)
        self.map_img = self.map_img.astype(np.int8)
        self.map_img = np.where((0 <= self.map_img) & (self.map_img < 100),
                                self.map_img + 1, self.map_img)
        self.pre_pose = msg.pose.pose


def main(args=None):
    rclpy.init(args=args)
    andante_camera_linetrace = AndanteCameraLinetrace()
    rclpy.spin(andante_camera_linetrace)
    andante_camera_linetrace.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
