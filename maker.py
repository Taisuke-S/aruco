# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# ２つの aruco マーカー id 0,1 を生成
# 
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

import cv2
import numpy as np

aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
marker_id = 0  # IDは0〜49の範囲で指定
marker_size = 200  # ピクセルサイズ

marker_img = cv2.aruco.generateImageMarker(aruco_dict, marker_id, marker_size)
cv2.imwrite("aruco_marker0.png", marker_img)
cv2.imshow("Marker0", marker_img)

marker_id =1  # IDは0〜49の範囲で指定

marker_img = cv2.aruco.generateImageMarker(aruco_dict, marker_id, marker_size)
cv2.imwrite("aruco_marker1.png", marker_img)
cv2.imshow("Marker1", marker_img)

cv2.waitKey(0)
cv2.destroyAllWindows()
