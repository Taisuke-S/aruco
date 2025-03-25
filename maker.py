import cv2
import numpy as np

aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
marker_id = 0  # IDは0〜49の範囲で指定
marker_size = 200  # ピクセルサイズ

marker_img = cv2.aruco.generateImageMarker(aruco_dict, marker_id, marker_size)
cv2.imwrite("aruco_marker.png", marker_img)
cv2.imshow("Marker", marker_img)
cv2.waitKey(0)
cv2.destroyAllWindows()
