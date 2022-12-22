import cv2
import numpy as np


def main(args=None):
    SIZE = 300
    EDGE = 15

    aruco = cv2.aruco
    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
    marker = aruco.drawMarker(aruco_dict, 0, SIZE)

    img = np.ones((SIZE + EDGE * 2, SIZE + EDGE * 2))
    img *= 255
    img[EDGE:SIZE+EDGE, EDGE:SIZE+EDGE] = marker
    cv2.imwrite('aruco_0.jpg', img)


if __name__ == '__main__':
    main()
