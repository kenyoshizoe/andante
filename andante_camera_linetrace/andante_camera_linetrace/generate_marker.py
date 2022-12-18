import cv2

def main(args=None):
    aruco = cv2.aruco
    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
    img = aruco.drawMarker(aruco_dict, 0, 150)
    cv2.imwrite('aruco_0.jpg', img)

if __name__ == '__main__':
    main()
