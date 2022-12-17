#! /usr/bin/env python3
import rclpy
import cv2
import glob
import numpy as np


def main(args=None):
    rclpy.init(args=args)

    n = rclpy.create_node("andante_camera_calibration")

    n.declare_parameter("img_path")
    n.declare_parameter("dst_path")
    n.declare_parameter("width")
    n.declare_parameter("height")
    n.declare_parameter("rows")
    n.declare_parameter("cols")
    n.declare_parameter("grid_size")

    rows = n.get_parameter("rows")
    cols = n.get_parameter("cols")
    grid_size = n.get_parameter("grid_size")

    chess_pattern = np.zeros((rows * cols, 3), np.float32)
    chess_pattern[:, :2] = np.mgrid[0:rows * grid_size:grid_size,
                                    0:cols*grid_size:grid_size].T.reshape(-1, 2)

    print(chess_pattern)

    img_path = n.get_parameter("img_path")

    obj_points = []
    img_points = []

    for path in glob.glob(img_path + '/*'):
        img = cv2.imread(path)
        if img is None:
            continue

        img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        ret, corners = cv2.findChessboardCorners(img, (rows, cols), None)

        if ret is None:
            continue

        criteria = (cv2.TERM_CRITERIA_EPS +
                    cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        corners2 = cv2.cornerSubPix(img, corners, (11, 11), (-1, -1), criteria)

        obj_points.append(chess_pattern)
        img_points.append(corners2)

    width = n.get_parameter("width")
    height = n.get_parameter("height")
    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
        obj_points, img_points, (width, height), None, None)

    dst_path = n.get_parameter("dst_path")

    rclpy.shutdown()

if __name__ == '__main__':
    main()
