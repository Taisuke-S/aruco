import cv2
import numpy as np
import aerotap
import time
from collections import deque

# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# cv2.aruco.estimatePoseSingleMarkers() で得られるマーカーコーナー2D座標から、
# カメラワールド座標3Dを求め、3Dコーナー座標から変換行列を求める
# 3D座標が求められない場合 = コーナーの距離座標が取得てきない場合 = 距離=0 は、無視
#
# 重要: 3Dカメラから得られる距離データ(Depth)にはブレが発生するため、移動平均化している
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

# 移動平均の管理
class DepthMovingAverage:
    def __init__(self, window_size):
        self.window_size = window_size
        self.values = deque(maxlen=window_size)  # 最大長を設定
        self.total = 0  # 合計値を保持（計算を高速化）

    def update(self, value):
        if len(self.values) == self.window_size:  
            self.total -= self.values[0]  # 古い値を減算
        self.values.append(value)
        self.total += value  # 新しい値を加算
    
    def get_average(self):
        if not self.values:
            return None  # データがない場合は None
        return self.total / len(self.values)

# マーカーコーナーの各距離を移動平均から得る
def getDepthAve(corners,depth):
   global nDepthSum,nDepthCount,depthMoving_averages

   nDepthAve = np.array([0,0,0,0])
   nDepth = getDepth(corners,depth)

   for i in range(depthMoving_averages):
       if nDepth[i]>0:
           depthMovingAverages[i].update(nDepth[i])
           nDepthAve[i] = depthMovingAverages[i].get_average()

   print(f"nDepth {nDepthAve[0]:.0f}, {nDepthAve[1]:.0f}, {nDepthAve[2]:.0f}, {nDepthAve[3]:.0f}")
   return nDepthAve


# マーカーコーナーの各距離を得る
def getDepth(corners,depth):
    
    global cameraWidth,cameraHeight

    nDepth = np.array([0,0,0,0])
    nIndex = np.array([-1,1,-cameraWidth,cameraWidth]) # aroung the point

    for i, point in enumerate(corners):
        x, y = point
        pos = int(y*cameraWidth+x)
        if 0 <= pos < depth.size: 
          nDepth[i] = depth[pos]
          if nDepth[i] ==0:
              for j, p in enumerate(nIndex):
#                  print(f"Index {j} => {nIndex[j]}")
                  nDepth[i] = depth[pos+nIndex[j]]
                  if nDepth[i] >0:
                      break
              
        else:
          nDepth[i] = 0
#        print(f"Index {i} => point: ({x},{y}):{depth[pos]}")
    return nDepth

# ピクセル座標に変換（プロジェクション後に正規化）
def normalize_projection(proj):
    if proj[2] != 0:
        return proj / proj[2]  # 正規化（x/z, y/z）
    else:
        print(f"Warning: Projection {proj.flatten()} has zero denominator")
        return np.array([0, 0])  # 代替値

#座標系の表示
def drawAxis(image,center,R):
    
    global cameraWidth,cameraHeight

    # カメラの内部パラメータ行列（カメラ行列）
    K = np.array([[50, 0, cameraWidth/2],
              [0, 50, cameraHeight/2],
              [0, 0, 1]])

    # 単位ベクトル（X, Y, Z 軸の基準）
    X_axis = np.array([1, 0, 0])  # X軸
    Y_axis = np.array([0, 1, 0])  # Y軸
    Z_axis = np.array([0, 0, 1])  # Z軸

    # 回転後の軸
    X_rotated = np.dot(R, X_axis)
    Y_rotated = np.dot(R, Y_axis)
    Z_rotated = np.dot(R, Z_axis)

    # カメラ行列 K を使って 3D 座標を 2D に投影
    X_projected = np.dot(K, X_rotated)
    Y_projected = np.dot(K, Y_rotated)
    Z_projected = np.dot(K, Z_rotated)    

    X_proj_norm = normalize_projection(X_projected)
    Y_proj_norm = normalize_projection(Y_projected)
    Z_proj_norm = normalize_projection(Z_projected)

# 座標変換の結果をチェック    if np.all(np.isfinite(X_proj_norm)):
    if np.all(np.isfinite(X_proj_norm)):
        X_point = (int(X_proj_norm[0]), int(X_proj_norm[1]))
    else:
        print("Error: X_projected contains invalid values:", X_proj_norm)
        X_point = (0, 0)  # エラー時のデフォルト値

    if np.all(np.isfinite(Y_proj_norm)):
        Y_point = (int(Y_proj_norm[0]), int(Y_proj_norm[1]))
    else:
        print("Error: Y_projected contains invalid values:", Y_proj_norm)
        Y_point = (0, 0)  # エラー時のデフォルト値

    if np.all(np.isfinite(Z_proj_norm)):
        Z_point = (int(Z_proj_norm[0]), int(Z_proj_norm[1]))
    else:
        print("Error: Z_projected contains invalid values:", Z_proj_norm)
        Z_point = (0, 0)  # エラー時のデフォルト値

    cent = (int(center[0][0]), int(center[0][1])) 
    # 回転後の座標系を描画（X軸は赤、Y軸は緑、Z軸は青）
    cv2.arrowedLine(image, cent, X_point, (0, 0, 255), 2, 8, 0, 0.05)  # X軸 (赤)
    cv2.arrowedLine(image, cent, Y_point, (0, 255, 0), 2, 8, 0, 0.05)  # Y軸 (緑)
    cv2.arrowedLine(image, cent, Z_point, (255, 0, 0), 2, 8, 0, 0.05)  # Z軸 (青)5);  // Z軸 (青)

# ArUcoマーカーの辞書を定義 (DICT_4X4_50 など他の辞書も使用可)
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
aruco_params = cv2.aruco.DetectorParameters()

cameraWidth = 640
cameraHeight = 480

# aeroTAP カメラを起動
if not aerotap.getCamModeAndType():
     raise ValueError("Camera detection Error")
# Initialize camera check
if not aerotap.Init(0,30,aerotap.camMode):
     raise ValueError("Error aeroInit")
   
#aerotap.EnableDepthFilter(bEnableDepthFilter)
if not aerotap.StartCam(cameraWidth,cameraHeight):
     raise ValueError("Camera Start Error")

print("Starting camera...type 'Esc' to terminate.")      

nFocalLength = np.array(
        [586.0000,586.0000]
    , dtype=np.float32)

nFocalLength[0], nFocalLength[1] = aerotap.GetFocalLength(0)
print(f"Focal Length: w={nFocalLength[0]}, h={nFocalLength[1]}")

# カメラ中心位置
camera_center = (cameraWidth/2, cameraHeight / 2)

cornerDepth = np.array([0,0,0,0])
point3D = np.array([0,0,0], dtype=np.float32)

marker_corners_3D = np.array([
        [0,0,0],
        [0,0,0],
        [0,0,0],
        [0,0,0]
    ], dtype=np.float32)

# 基準座標系の4つの対応する点（仮に設定）
# 基準座標系の対応点も同様に指定します ( 5cm x 5cm 50cm distance )
base_3D_corners = np.array([
    [-50, -50, 500],  # 左上
    [50, -50, 500],  # 右上
    [50, 50, 500],  # 右下
    [-50, 50, 500]   # 左下
], dtype=np.float32)

# マーカーの中心座標を計算
center_base = np.mean(base_3D_corners, axis=0)

camera_matrix = np.array([[1000, 0, 640], 
                          [0, 1000, 360], 
                          [0, 0, 1]], dtype=np.float32) 
dist_coeffs = np.zeros((4, 1))  # 歪み係数（例）

# === 4つのDepth移動平均を管理 ===
depthAve_size = 10
depthMoving_averages = 4
depthMovingAverages = [DepthMovingAverage(depthAve_size) for _ in range(depthMoving_averages)]

nViewMode =0
nInvertMode =0

# カメラメイン ループ
while True:
  if aerotap.IsNewFrame():

    frame = aerotap.ReadImage(7)
    depth = aerotap.ReadDepth()

    # convert depth map to bgr ColorImage
    if( not (depth is None) ):
       imgDepth = aerotap.DepthToRGB(depth)
       cv2.imshow( "aeroTAP camera DepthMap", imgDepth )

    aerotap.UpdateFrame()

    # グレースケールに変換
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # マーカーを検出
    detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)
    corners, ids, _ = detector.detectMarkers(gray)

    # マーカーが見つかった場合
    if ids is not None:

        marker_size = 0.05  # 5cmのマーカーと仮定
        rvecs2D, tvecs2D, _ = cv2.aruco.estimatePoseSingleMarkers(corners, marker_size, camera_matrix, dist_coeffs)

        cv2.aruco.drawDetectedMarkers(frame, corners, ids)
        for i, rvec in enumerate(rvecs2D):
        # 回転ベクトルを回転行列に変換
            R, _ = cv2.Rodrigues(rvec)

            # マーカーの座標系を描画
            if nViewMode ==1:
               cv2.drawFrameAxes(frame, camera_matrix, dist_coeffs, rvec, tvecs2D[i], 0.03)

#        print("cornsers: ", corners) 
        marker_corners_2D = corners[0].reshape(4, 2) 
        center_marker = np.mean(marker_corners_2D, axis=1)
#        print("center_marker: ", center_marker) 
        #マーカー位置の距離 
        cornerDepth = getDepthAve(marker_corners_2D,depth)
        # 距離が取得出来ていない場合
        if np.any(cornerDepth <= 0):
            continue

#        point3D = np.zeros(3, dtype=np.float32)
        #マーカーコーナー位置を3D座標に変換 
        # arucoの座標系に合わせるためにx軸の符号を反転？
        corners_array = corners[0]
        for i, point in enumerate(corners_array[0]):
              x,y = point
              point3D = np.array([
               -(x - camera_center[0]) * cornerDepth[i] / nFocalLength[0],
               (y - camera_center[1]) * cornerDepth[i] / nFocalLength[1],
               cornerDepth[i]
               ], dtype=np.float32)
              marker_corners_3D[i]  = point3D            

        # マーカーの中心座標を計算
        center_marker3D = np.mean(marker_corners_3D, axis=0)

        # 座標系を中心からの相対的な位置に変換
        P1 = marker_corners_3D - center_marker3D  # マーカーの相対座標系
        P2 = base_3D_corners - center_base  # 基準座標系の相対座標系

        # 3. SVDを使用して回転行列を求める
        # H 行列を計算 (P2.T * P1)
        H = np.dot(P2.T, P1)

        # SVDを適用
        U, S, Vt = np.linalg.svd(H)

        # 回転行列 R の計算
        R = np.dot(Vt.T, U.T)
        # 行列式をチェックして反射行列でないか確認
        det_R = np.linalg.det(R)
 #       print("行列式 det(R):", det_R)

        # もし行列式が -1 なら、反射行列なので修正します
        if det_R < 0:
             Vt[-1, :] *= -1
             R = np.dot(Vt.T, U.T)
        rvec3D, _ = cv2.Rodrigues(R)  # 3x3 → 3x1 の回転ベクトル

#        print("Rotation matrix R:\n", R)"
        # マーカーの座標系を描画
        if nViewMode ==0:
            cv2.drawFrameAxes(frame, camera_matrix, dist_coeffs, rvec3D, tvecs2D[0], 0.03)
#        drawAxis(frame,center_marker,R)

        for i, corner in enumerate(corners):
            x, y= int(corner[0][0][0]), int(corner[0][0][1])  # 左上の座標
            marker_id = int(ids[i][0])
            cv2.putText(frame, f"ID: {marker_id}", (x, y - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            cv2.putText(frame, f"{x},{y}", (10, 10+i*20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        for i, corner3D in enumerate(marker_corners_3D):
            x, y,z = corner3D
            cv2.putText(frame, f"{i}:{x:.1f},{y:.1f},{z:.1f}", (10, 100+i*20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
    # 結果を表示
    cv2.imshow("Aruco Marker Detection", frame)

    # ESCキーで終了
    key = cv2.waitKey(1)
    if key & 0xFF == 27:
        break
    if key & 0xFF == ord("v"):
        if nViewMode == 0:
            nViewMode =1
        else:
            nViewMode =0
    if key & 0xFF == ord("w"):
        if nInvertMode == 0:
            nInvertMode =1
        else:
            nInvertMode =0
  else:
    err = aerotap.GetLastError()
#   ant camera error?
    if  err:
        break
  time.sleep(0.033)

#cap.release()
aerotap.Close()

cv2.destroyAllWindows()
