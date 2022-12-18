import rclpy
import cv2
import math
import numpy as np


def main(args=None):
    rclpy.init(args=args)

    n = rclpy.create_node("andante_camera_linetrace")

    n.declare_parameter("camera.src")
    n.declare_parameter("camera.width")
    n.declare_parameter("camera.height")
    n.declare_parameter("camera.fps")
    n.declare_parameter("camera.mode")
    n.declare_parameter("camera.camera_matrix.fx")
    n.declare_parameter("camera.camera_matrix.fy")
    n.declare_parameter("camera.camera_matrix.cx")
    n.declare_parameter("camera.camera_matrix.cy")
    n.declare_parameter("camera.dist")
    n.declare_parameter("camera.M")
    n.declare_parameter("map.length_x")
    n.declare_parameter("map.length_y")
    n.declare_parameter("map.resolution")
    n.declare_parameter("map.robot_pose.x")
    n.declare_parameter("map.robot_pose.y")
    n.declare_parameter("map.robot_pose.theta")
    n.declare_parameter("calibration.src")
    n.declare_parameter("calibration.marker_size")
    n.declare_parameter("calibration.marker_pose.x")
    n.declare_parameter("calibration.marker_pose.y")
    n.declare_parameter("calibration.marker_pose.theta")

    # load img
    img_path = n.get_parameter("calibration.src").value
    img = cv2.imread(img_path)
    if img is None:
        n.get_logger().warn("couldnt load img. path: {}".format(img_path))
        return

    # load camera matrix
    fx = n.get_parameter("camera.camera_matrix.fx").value
    fy = n.get_parameter("camera.camera_matrix.fy").value
    cx = n.get_parameter("camera.camera_matrix.cx").value
    cy = n.get_parameter("camera.camera_matrix.cy").value
    camera_matrix = np.array([
        [fx, 0, cx],
        [0, fy, cy],
        [0, 0, 1]
    ])
    dist = np.array(n.get_parameter("camera.dist").value)

    # load map settings
    map_x = n.get_parameter("map.length_x").value
    map_y = n.get_parameter("map.length_y").value
    map_resolution = n.get_parameter("map.resolution").value

    # undistort
    img = cv2.undistort(img, camera_matrix, dist)
    camera_matrix = cv2.getOptimalNewCameraMatrix(
        camera_matrix, dist, (img.shape[0], img.shape[1]), 0.0)

    # find aruco marker
    aruco = cv2.aruco
    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
    img_corners, ids, rejectedImgPoints = aruco.detectMarkers(img, aruco_dict)
    img_corners = np.array(img_corners).reshape(-1, 2).astype(np.float32)

    # calc marker pose in map coord
    marker_size = n.get_parameter("calibration.marker_size").value
    marker_x = n.get_parameter("calibration.marker_pose.x").value
    marker_y = -n.get_parameter("calibration.marker_pose.y").value
    marker_theta = -n.get_parameter("calibration.marker_pose.theta").value
    robot_x = n.get_parameter("map.robot_pose.x").value
    robot_y = n.get_parameter("map.robot_pose.y").value
    robot_theta = n.get_parameter("map.robot_pose.theta").value

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
    print(M)

    # preview
    size = (int(map_y / map_resolution), int(map_x / map_resolution))
    preview = cv2.warpPerspective(img, M, size)
    cv2.imshow("preview", preview)
    cv2.waitKey(0)


if __name__ == '__main__':
    main()
