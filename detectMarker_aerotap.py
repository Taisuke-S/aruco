import cv2
import numpy as np
import aerotap

# ArUcoマーカーの辞書を定義 (DICT_4X4_50 など他の辞書も使用可)
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
aruco_params = cv2.aruco.DetectorParameters()

def drawreMarkerRect(image,corners,ids):
# 線の色と太さの指定
    print("drawreMarkerRect called")
    line_color = (255, 255, 255)  # 赤 (BGR0形式)
    line_thickness = 4
    if corners is None or len(corners) == 0:
        print("No corners detected")
        return
# 各マーカーに対して枠を描画
    for i, corner in enumerate(corners):
        pts = corner[0].astype(int)  # 4点の座標 [4, 2]
        cv2.polylines(image, [pts], isClosed=True, color=line_color, thickness=line_thickness)
        print(pts)

    # オプション：マーカーIDを表示
        if ids is not None:
            center = np.mean(pts, axis=0).astype(int)
            cv2.putText(image, f"ID {ids[i][0]}", tuple(center), cv2.FONT_HERSHEY_SIMPLEX,
                0.5, (0, 0, 255), 2, cv2.LINE_AA)

# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# メイン処理
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#
# カメラを起動
#cap = cv2.VideoCapture(0)
if not aerotap.getCamModeAndType():
     raise ValueError("Camera detection Error")
# Initialize camera 
if not aerotap.Init(0,30,aerotap.camMode):
     raise ValueError("Error aeroInit")
   
#aerotap.EnableDepthFilter(bEnableDepthFilter)
if not aerotap.StartCam(1280,720):
     raise ValueError("Camera Start Error")

print("Starting camera...type 'Esc' to terminate.")      

camera_matrix = np.array([[1000, 0, 640], 
                          [0, 1000, 360], 
                          [0, 0, 1]], dtype=np.float32) 
dist_coeffs = np.zeros((4, 1))  # 歪み係数（例）

while True:
  if aerotap.IsNewFrame():
    frame = aerotap.Read(0)
    depth = aerotap.ReadDepth()  # depth map 16 bit
#          depth = ReadDepth()
    aerotap.UpdateFrame()

    # グレースケールに変換
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    if( not (depth is None) ):
       # convert depth map to bgr ColorImage
       imgDepth = aerotap.DepthToRGB(depth)

    # マーカーを検出
    detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)
    corners, ids, _ = detector.detectMarkers(gray)

    # マーカーが見つかった場合
    if ids is not None:
        
        marker_size = 0.05  # 5cmのマーカーと仮定
        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, marker_size, camera_matrix, dist_coeffs)

        drawreMarkerRect(imgDepth,corners,ids)

        cv2.aruco.drawDetectedMarkers(frame, corners, ids)
        for i, rvec in enumerate(rvecs):
        # 回転ベクトルを回転行列に変換
            R, _ = cv2.Rodrigues(rvec)

            print(f"Marker ID {ids[i][0]}")
            print("Rotation Matrix:\n", R)
            print("Translation Vector:\n", tvecs[i])

        # マーカーの座標系を描画
            cv2.drawFrameAxes(frame, camera_matrix, dist_coeffs, rvec, tvecs[i], 0.03)

        for i, corner in enumerate(corners):
            x, y = int(corner[0][0][0]), int(corner[0][0][1])  # 左上の座標
            marker_id = int(ids[i][0])
            cv2.putText(frame, f"ID: {marker_id}", (x, y - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            cv2.putText(frame, f"#{i}: {x},{y}", (10, 10+i*20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
    # 結果を表示
    cv2.imshow("Aruco Marker Detection", frame)

    if( not (depth is None) ):
       cv2.imshow( "aeroTAP camera DepthMap", imgDepth )

    # ESCキーで終了
    if cv2.waitKey(1) & 0xFF == 27:
        break
  else:
    err = aerotap.GetLastError()
#   ant camera error?
    if  err:
        break
#cap.release()
aerotap.Close()

cv2.destroyAllWindows()
