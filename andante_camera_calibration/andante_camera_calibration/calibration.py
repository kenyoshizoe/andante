#! /usr/bin/env python3
import rclpy
import cv2
import glob
import yaml
import numpy as np


def main(args=None):
    rclpy.init(args=args)

    n = rclpy.create_node("andante_camera_calibration")

    n.declare_parameter("preview")
    n.declare_parameter("img_path")
    n.declare_parameter("dst_path")
    n.declare_parameter("rows")
    n.declare_parameter("cols")
    n.declare_parameter("grid_size")

    preview = n.get_parameter("preview").value
    rows = n.get_parameter("rows").value
    cols = n.get_parameter("cols").value
    grid_size = n.get_parameter("grid_size").value

    chess_pattern = np.zeros((rows * cols, 3), np.float32)
    chess_pattern[:, :2] = np.mgrid[0:rows * grid_size:grid_size,
                                    0:cols*grid_size:grid_size].T.reshape(-1, 2)

    img_path = n.get_parameter("img_path").value

    width = -1
    height = -1
    obj_points = []
    img_points = []

    for path in glob.glob(img_path + '/*'):
        img = cv2.imread(path)
        if img is None:
            continue
        if width == -1 or height == -1:
            height = img.shape[0]
            width = img.shape[1]

        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        ret, corners = cv2.findChessboardCorners(gray, (rows, cols), None)

        if not ret:
            continue

        criteria = (cv2.TERM_CRITERIA_EPS +
                    cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        corners2 = cv2.cornerSubPix(
            gray, corners, (11, 11), (-1, -1), criteria)

        if preview:
            cv2.imshow("preview - hit any key to proceed",
                       cv2.drawChessboardCorners(img, (rows, cols), corners2, None))
            key = cv2.waitKey(0)
            if key == 'n':
                continue

        obj_points.append(chess_pattern)
        img_points.append(corners2)

    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
        obj_points, img_points, (width, height), None, None)

    dst_path = n.get_parameter("dst_path").value
    with open(dst_path, 'w') as f:
        result = {
            'width': width,
            'height': height,
            'camera_matrix': {
                'fx': float(mtx[0][0]),
                'fy': float(mtx[1][1]),
                'cx': float(mtx[0][2]),
                'cy': float(mtx[1][2])
            },
            'distCoeffs': dist.tolist()
        }
        yaml.dump(result, f)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
