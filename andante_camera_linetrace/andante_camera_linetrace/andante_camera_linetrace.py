import rclpy
import cv2
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import Header
import tf_transformations
import tf2_ros
import math


class AndanteCameraLinetrace(Node):
    def __init__(self):
        super().__init__("andante_camera_linetrace")
        self.declare_params()
        self.param_loaded = False
        self.image_received = False

        self.M = np.array(self.get_parameter("camera.M").value).reshape(3, 3)
        self.threshold = self.get_parameter("camera.threshold").value
        if self.get_parameter("camera.use_topic"):
            self.img_sub = self.create_subscription(
                Image, "camera/image_raw",
                self.img_callback, 10
            )
            self.camera_info_sub = self.create_subscription(
                CameraInfo, "camera/camera_info",
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

        # setup map
        map_size = self.get_parameter("map.size").value
        self.map_resolution = self.get_parameter("map.resolution").value
        self.map_img = np.full(
            (int(map_size / self.map_resolution), int(map_size / self.map_resolution)), 100, dtype=np.int8)
        self.map_pub = self.create_publisher(
            OccupancyGrid, "map", 10
        )
        self.map_msg = OccupancyGrid()
        self.map_msg.header = Header()
        self.map_msg.header.frame_id = self.get_parameter("base_frame").value
        self.map_msg.info.resolution = self.map_resolution
        self.map_msg.info.width = int(map_size / self.map_resolution)
        self.map_msg.info.height = int(map_size / self.map_resolution)
        self.map_msg.info.origin.position.x = - map_size / 2
        self.map_msg.info.origin.position.y = - map_size / 2
        # setup map updator
        self.create_subscription(
            Odometry, "odom", self.map_update, 10
        )
        self.pre_pose = None
        self.map_trans_tolerance = self.get_parameter(
            "map.trans_tolerance").value
        self.map_rotate_tolerance = self.get_parameter(
            "map.rotate_tolerance").value

        self.motion_controller = self.create_timer(0.1, self.motion_control)
        self.twis_pub = self.create_publisher(
            Twist, "cmd_vel", 10
        )
        self.center_circle_radius = self.get_parameter(
            "motion_control.center_circle_radius").value // self.map_resolution
        self.max_linear_vel = self.get_parameter(
            "motion_control.max_linear_vel").value
        self.min_linear_vel = self.get_parameter(
            "motion_control.min_linear_vel").value
        self.angular_vel_scale = self.get_parameter(
            "motion_control.angular_vel_scale").value

    def declare_params(self):
        self.declare_parameter("base_frame")
        self.declare_parameter("camera.use_topic")
        self.declare_parameter("camera.img_topic")
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
        self.declare_parameter("camera.threshold")
        self.declare_parameter("map.size")
        self.declare_parameter("map.resolution")
        self.declare_parameter("map.trans_tolerance")
        self.declare_parameter("map.rotate_tolerance")
        self.declare_parameter("motion_control.center_circle_radius")
        self.declare_parameter("motion_control.max_linear_vel")
        self.declare_parameter("motion_control.min_linear_vel")
        self.declare_parameter("motion_control.angular_vel_scale")
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
        ret, frame = cv2.threshold(frame, 200, 100, cv2.THRESH_BINARY_INV)

        frame += 1
        observe = cv2.warpPerspective(
            frame, self.M, self.map_img.shape, borderMode=cv2.BORDER_CONSTANT, borderValue=-1).astype(np.int8)
        observe -= 1

        self.map_img = np.where(
            observe == -1, self.map_img, observe).astype(np.uint8)

        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
        self.map_img = cv2.morphologyEx(
            self.map_img, cv2.MORPH_CLOSE, kernel, iterations=1)

        cv2.circle(
            self.map_img, (self.map_img.shape[1] // 2, self.map_img.shape[0] // 2), 5, 100, -1)
        # publish map_msg
        self.map_msg.header.stamp = self.get_clock().now().to_msg()
        map_data = cv2.flip(self.map_img, 0).astype(np.int8).flatten().tolist()
        self.map_msg.data = map_data
        self.map_pub.publish(self.map_msg)

        self.image_received = True

    def map_update(self, msg):
        if self.pre_pose is None:
            self.pre_pose = msg.pose.pose

        # find map transform
        dx = msg.pose.pose.position.x - self.pre_pose.position.x
        dy = msg.pose.pose.position.y - self.pre_pose.position.y
        pre_theta = tf_transformations.euler_from_quaternion([
            msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])[2]
        cur_theta = tf_transformations.euler_from_quaternion([
            self.pre_pose.orientation.x, self.pre_pose.orientation.y,
            self.pre_pose.orientation.z, self.pre_pose.orientation.w])[2]
        dtheta = -(cur_theta - pre_theta)
        if abs(math.sqrt(dx**2 + dy**2)) < self.map_trans_tolerance and abs(dtheta) < self.map_rotate_tolerance:
            return
        dx_img = (math.cos(cur_theta) * dx +
                  math.sin(cur_theta) * dy) / self.map_resolution
        dy_img = (-math.sin(cur_theta) * dx +
                  math.cos(cur_theta) * dy) / self.map_resolution
        M_trans = np.array([
            [1, 0, -dx_img],
            [0, 1, -dy_img]
        ]).astype(np.float32)
        M_rotate = cv2.getRotationMatrix2D(
            (self.map_img.shape[1] / 2, self.map_img.shape[0]/2), -dtheta/math.pi*180, 1.0)

        # transfrom map
        self.map_img = self.map_img.astype(np.float32)
        self.map_img = cv2.warpAffine(
            self.map_img, M_trans, self.map_img.shape, flags=cv2.INTER_NEAREST, borderValue=100)
        self.map_img = cv2.warpAffine(
            self.map_img, M_rotate, self.map_img.shape, flags=cv2.INTER_NEAREST, borderValue=100)
        # remove noise
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
        self.map_img = cv2.morphologyEx(
            self.map_img, cv2.MORPH_CLOSE, kernel, iterations=1)
        # gradually reduce information
        self.map_img = np.where((self.map_img < 100),
                                self.map_img + 1, self.map_img)
        self.pre_pose = msg.pose.pose

        cv2.circle(
            self.map_img, (self.map_img.shape[1] // 2, self.map_img.shape[0] // 2), 5, 100, -1)

    def motion_control(self):
        if not self.image_received:
            return
        mesh_x, mesh_y = np.meshgrid(
            range(self.map_img.shape[0]), range(self.map_img.shape[1]))
        mesh_x = mesh_x.astype(np.float64)
        mesh_y = mesh_y.astype(np.float64)

        mesh_x -= self.map_img.shape[0] / 2
        mesh_y -= self.map_img.shape[1] / 2
        length = (mesh_x ** 2 + mesh_y ** 2) ** 0.5
        cost = self.map_img + (length + 10)
        cost = cv2.blur(cost, (5, 5))

        min_idx = np.unravel_index(np.argmin(cost), cost.shape)
        ref_theta = math.atan2(
            min_idx[0] - self.map_img.shape[1] / 2, min_idx[1] - self.map_img.shape[0] / 2)

        mask = np.zeros(self.map_img.shape, np.uint8)
        mask = cv2.line(
            mask,
            (int(self.map_img.shape[0] / 2), int(self.map_img.shape[1] // 2)),
            (int(self.map_img.shape[0] * math.cos(ref_theta) + self.map_img.shape[1] / 2),
             int(self.map_img.shape[0] * math.sin(ref_theta) + self.map_img.shape[0] / 2)),
            255, 3
        )
        mean = 100 - cv2.mean(self.map_img, mask=mask)[0]
        twist_msg = Twist()
        twist_msg.linear.x = max(
            mean * (self.max_linear_vel / 100.0), self.min_linear_vel)
        twist_msg.angular.z = -ref_theta * self.angular_vel_scale
        self.twis_pub.publish(twist_msg)


def main(args=None):
    rclpy.init(args=args)
    andante_camera_linetrace = AndanteCameraLinetrace()
    rclpy.spin(andante_camera_linetrace)
    andante_camera_linetrace.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
