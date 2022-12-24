import rclpy
from rclpy.node import Node
import cv2
import math
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CameraInfo


class AndanteCameraLinetraceCalibration(Node):
    def __init__(self):
        super().__init__("andante_camera_linetrace")
        self.declare_params()
        self.loaded_param = False

        if self.get_parameter("camera.use_topic").value:
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
            self.load_params()
            self.loaded_param = True
            img_path = self.get_parameter("calibration.src").value
            img = cv2.imread(img_path)
            if img is None:
                self.get_logger().warn("couldnt load img. path: {}".format(img_path))
                raise SystemExit
                return
            # undistort
            undistort_image = cv2.undistort(
                img, self.camera_matrix, self.distcoeffs)
            self.camera_matrix = cv2.getOptimalNewCameraMatrix(
                self.camera_matrix, self.distcoeffs, (img.shape[0], img.shape[1]), 0.0)
            self.calibration(undistort_image)

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

    def load_params(self):
        # load camera matrix
        fx = self.get_parameter("camera.camera_matrix.fx").value
        fy = self.get_parameter("camera.camera_matrix.fy").value
        cx = self.get_parameter("camera.camera_matrix.cx").value
        cy = self.get_parameter("camera.camera_matrix.cy").value
        self.camera_matrix = np.array([
            [fx, 0, cx],
            [0, fy, cy],
            [0, 0, 1]
        ]).astype(np.float32)
        self.distcoeffs = np.array(self.get_parameter(
            "camera.dist").value).astype(np.float32)

    def img_callback(self, msg):
        if self.loaded_param == False:
            return
        frame = None
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            print(e)
            return
        undistort_frame = cv2.undistort(
            frame, self.camera_matrix, self.distcoeffs)
        self.camera_matrix = cv2.getOptimalNewCameraMatrix(
            self.camera_matrix, self.distcoeffs, (undistort_frame.shape[0], undistort_frame.shape[1]), 0.0)
        self.calibration(undistort_frame)

    def camera_info_callback(self, msg):
        if self.loaded_param == False:
            self.camera_matrix = np.array(msg.k).reshape(3, 3)
            self.distcoeffs = np.array(msg.d).reshape(-1)
            self.loaded_param = True

    def calibration(self, img):
        # find aruco marker
        aruco = cv2.aruco
        aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
        img_corners, ids, rejectedImgPoints = aruco.detectMarkers(
            img, aruco_dict)
        img_corners = np.array(img_corners).reshape(-1, 2).astype(np.float32)
        if img_corners.shape[0] == 0:
            self.get_logger().error('Couldn\'t find aruco marker.')
            raise SystemExit

        # calc marker pose in map coord
        marker_size = self.get_parameter("calibration.marker_size").value
        marker_x = self.get_parameter("calibration.marker_pose.x").value
        marker_y = -self.get_parameter("calibration.marker_pose.y").value
        marker_theta = - \
            self.get_parameter("calibration.marker_pose.theta").value

        map_size = self.get_parameter("map.size").value
        map_resolution = self.get_parameter("map.resolution").value

        robot_x = map_size / 2
        robot_y = map_size / 2
        robot_theta = 0

        t1 = np.matrix([
            [math.cos(robot_theta), -math.sin(robot_theta), robot_x],
            [math.sin(robot_theta), math.cos(robot_theta), robot_y],
            [0, 0, 1]
        ])
        t2 = np.matrix([
            [math.cos(marker_theta), -math.sin(marker_theta), marker_x],
            [math.sin(marker_theta), math.cos(marker_theta), marker_y],
            [0, 0, 1]
        ])

        map_corners = np.array([
            [marker_size / 2, marker_size/2],
            [-marker_size / 2, marker_size / 2],
            [-marker_size / 2, -marker_size / 2],
            [marker_size / 2, -marker_size / 2]
        ])

        for i, corner in enumerate(map_corners):
            p = np.matrix([corner[0], corner[1], 1]).T
            p = t1 * t2 * p
            corner = p.T[0, 0:2]
            corner /= map_resolution
            map_corners[i] = corner
        map_corners = map_corners.reshape(-1, 2).astype(np.float32)

        M = cv2.getPerspectiveTransform(img_corners, map_corners)
        print(M.flatten().tolist())

        # preview
        size = (int(map_size / map_resolution), int(map_size / map_resolution))
        preview = cv2.warpPerspective(img, M, size)
        preview = cv2.polylines(preview, map_corners.reshape(1, -1, 2).astype(np.int32), True,
                                (255, 0, 0), thickness=1)
        cv2.imshow("preview", preview)
        cv2.waitKey(0)
        raise SystemExit


def main(args=None):
    rclpy.init(args=args)
    andante_camera_linetrace_calibration = AndanteCameraLinetraceCalibration()
    try:
        rclpy.spin(andante_camera_linetrace_calibration)
    except SystemExit:
        rclpy.logging.get_logger("Quitting").info('Done')
    rclpy.shutdown()


if __name__ == '__main__':
    main()
