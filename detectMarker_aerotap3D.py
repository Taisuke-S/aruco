import cv2
import numpy as np
import aerotap
import time
from collections import deque
from scipy.spatial.transform import Rotation

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
          nDepth[i] = depth[pos];
          if nDepth[i] ==0:
              for j, p in enumerate(nIndex):
#                  print(f"Index {j} => {nIndex[j]}")
                  nDepth[i] = depth[pos+nIndex[j]]
                  if nDepth[i] >0:
                      break
              
        else:
          nDepth[i] = 0
#    print(f"nDepth {nDepth[0]:.0f}, {nDepth[1]:.0f}, {nDepth[2]:.0f}, {nDepth[3]:.0f}")
    return nDepth

    """ 4x4 変換行列の逆行列を計算 """
def invert_transformation_matrix(T):
    R = T[:3, :3]  # 回転行列 (3x3)
    t = T[:3, 3]   # 並進ベクトル (3x1)

    # 逆変換: R^T と -R^T t を計算
    R_inv = R.T
    t_inv = -np.dot(R_inv, t)

    # 新しい逆行列を作成
    T_inv = np.eye(4)  # 単位行列を作成
    T_inv[:3, :3] = R_inv
    T_inv[:3, 3] = t_inv

    return T_inv

# カメラからマーカーへの方向ベクトルを 3D カメラの座標系で求める
"""
    カメラ座標系の4点 (x, y, z) からマーカーの変換行列 (4x4) を求める
"""
# marker_corners_3D は (4,3) の配列で、それぞれの点が (x, y, z) で表される。
# step1. 平面フィッティング (主成分分析: PCA)
#        マーカーは平面上にあるため、4点の位置から「平面の法線ベクトル」を求め、姿勢を決定。
#        座標軸の定義
#       x 軸: 1つ目の点と2つ目の点のベクトル
#       y 軸: x 軸と法線の外積
#       z 軸: 法線ベクトル (外積で求める)
#       変換行列を作成
# step2 回転行列 (3×3) は x, y, z 軸を並べて作成。
# step3 並進ベクトル (t) は、マーカーの中心位置。

def compute_transformation_matrix(marker_corners_3D):

    # (4,3) の形状を確認
    assert marker_corners_3D.shape == (4, 3), "Input shape must be (4,3)"
    
    # 1. 中心座標を計算 (マーカーの中心)
    center = np.mean(marker_corners_3D, axis=0)

    # 2. 基準となるX軸の方向を決定 (1つ目と2つ目の点を使用)
    x_axis = marker_corners_3D[1] - marker_corners_3D[0]
    x_axis /= np.linalg.norm(x_axis)  # 正規化

    # 3. 平面の法線ベクトル (z軸) を求める
    normal = np.cross(marker_corners_3D[2] - marker_corners_3D[0], 
                      marker_corners_3D[3] - marker_corners_3D[0])
    normal /= np.linalg.norm(normal)  # 正規化

    # 4. y軸を計算 (z軸とx軸の外積)
    y_axis = np.cross(normal, x_axis)
    y_axis /= np.linalg.norm(y_axis)  # 正規化

    # 5. 変換行列を作成
    R = np.column_stack((x_axis, y_axis, normal))  # 3x3 回転行列
    T = np.eye(4)  # 4x4 の単位行列
    T[:3, :3] = R  # 回転部分
    T[:3, 3] = center  # 平行移動部分

    return T


# 鏡映変換（左右反転）を含んでしまう場合 det_R<0
def adjustTransformationMatrix(T):
        
        # 回転行列 (3x3)
        R = T[:3, :3]
        # 並進ベクトル (3x1)
        t = T[:3, 3]

        det_R = np.linalg.det(R)
        if det_R <0:
        # 特異値分解 (SVD)
            U, _, Vt = np.linalg.svd(R)
        # 修正: det(U @ Vt) < 0 の場合、Vt の最後の行の符号を反転
            if np.linalg.det(U @ Vt) < 0:
                 Vt[-1, :] *= -1  # Vt の最後の行を反転
        # 正しい回転行列を取得
                 R = U @ Vt

        # 4×4 の変換行列 T を再度作成
        T = np.eye(4)  # 4×4 の単位行列
        T[:3, :3] = R  # 直行化した回転行列を適用
        T[:3,  3] = t  # 元の平行移動ベクトルをセット
        return T

# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# メイン処理
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#
cameraWidth = 640
cameraHeight = 480
# カメラ内部を90として、30FPSで処理するには  nFPS = 190 とする
nFPS = 30
# ArUcoマーカーの辞書を定義 (DICT_4X4_50 など他の辞書も使用可)
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
aruco_params = cv2.aruco.DetectorParameters()

# カメラを起動
if not aerotap.getCamModeAndType():
     raise ValueError("Camera detection Error")
# Initialize camera 
if not aerotap.Init(0,nFPS,aerotap.camMode):
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

camera_matrix = np.array([[1000, 0, 640], 
                          [0, 1000, 360], 
                          [0, 0, 1]], dtype=np.float32) 
dist_coeffs = np.zeros((4, 1))  # 歪み係数（例）

# コーナーの距離
cornerDepth = np.array([0,0,0,0])
point3D = np.array([0,0,0], dtype=np.float32)
marker_center_3D = np.array([0,0,0], dtype=np.float32)

# コーナーの3D座標
marker_corners_3D = np.array([
        [0,0,0],
        [0,0,0],
        [0,0,0],
        [0,0,0]
    ], dtype=np.float32)

marker_corners_2D = np.array([
        [0,0,0],
        [0,0,0],
        [0,0,0],
        [0,0,0]
    ], dtype=np.int32)

# 基準座標系の4つの対応する点（仮に設定）
# 基準座標系の対応点も同様に指定します ( 5cm x 5cm 50cm distance )
# カメラとアーム先端のオフセットとして利用可能?
base_3D_corners = np.array([
    [-50, -50, 0],  # 左上
    [50, -50, 0],  # 右上
    [50, 50, 0],  # 右下
    [-50, 50, 0]   # 左下
], dtype=np.float32)

# === 4つのDepth移動平均を管理 ===
depthAve_size = 10
depthMoving_averages = 4
depthMovingAverages = [DepthMovingAverage(depthAve_size) for _ in range(depthMoving_averages)]

# マーカーの中心座標を計算(カメラの中心位置)
center_base = np.mean(base_3D_corners, axis=0)
nViewMode =0
nInvertMode =0

while True:
  if aerotap.IsNewFrame():

    frame = aerotap.ReadImage(7)
    depth = aerotap.ReadDepth()
    # convert depth map to bgr ColorImage
    if( not (depth is None) ):
       imgDepth = aerotap.DepthToRGB(depth)

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

#            print(f"Marker ID {ids[i][0]}")
            # マーカーの座標系を描画
            if nViewMode ==1:
               cv2.drawFrameAxes(frame, camera_matrix, dist_coeffs, rvec, tvecs2D[i], 0.03)

        # corners[0] の 4 点の座標 (2D)
        marker_corners_2D = corners[0].reshape(4, 2)  # (4,2) の形に変換
#        marker_corners_3D = np.array(marker_corners_3D)  # (4,3) の形状
        # corners[0] の 4 点の距離(Z)
        cornerDepth = getDepthAve(marker_corners_2D,depth)

        # if Depth includes any valid 
        if np.any(cornerDepth <= 0) or nViewMode ==1:
            if nViewMode ==0:
               cv2.putText(frame, "Invalid corners depth value!", (10, 38),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
        else:
            #マーカーコーナー位置を3D座標に変換 
            # arucoの座標系に合わせるためにx軸の符号を反転？
            for i, point in enumerate(marker_corners_2D):
                x,y = point
                point3D = np.array([
                -(x - camera_center[0]) * cornerDepth[i] / nFocalLength[0],
                (y - camera_center[1]) * cornerDepth[i] / nFocalLength[1],            
                cornerDepth[i]
                ], dtype=np.float32)
    #              print(f"#{i} 3Dx {point3D[0]:.1f} = ({x}-{camera_center[0]}) * {cornerDepth[i] } / {nFocalLength[0]:.1f} ")
    #              print(f"#{i} 3Dy {point3D[1]:.1f} = ({y}-{camera_center[1]}) * {cornerDepth[i] } / {nFocalLength[1]:.1f} ")
                marker_corners_3D[i]  = point3D

            # マーカー中央の3D カメラの座標
            marker_center_3D = np.mean(marker_corners_3D, axis=0)  # (X, Y, Z)
            # カメラからマーカーへの方向ベクトルを 3D カメラの座標系で求め、新しい回転行列を得る
            T3D = compute_transformation_matrix(marker_corners_3D)

            # マーカーらからの変換行列
            if nInvertMode ==1:
               T3D = invert_transformation_matrix(T3D)

            # 4x4 変換行列から回転行列（R）と並進ベクトル（t）の取得
            # 回転行列 (3x3)
            R3D = T3D[:3, :3]
            # 並進ベクトル (3x1)
            t2D = T3D[:3, 3]
            #print("Rotation Matrix (R):\n", R)
            #print("Translation Vector (t):\n", t)
            rvec3D, _ = cv2.Rodrigues(R3D)  # 3x3 → 3x1 の回転ベクトル

            if nViewMode ==0:
              cv2.drawFrameAxes(frame, camera_matrix, dist_coeffs, rvec3D, tvecs2D[0], 0.03)

            # 回転行列 R からオイラー角（roll, pitch, yaw）を取得
            rot = Rotation.from_matrix(R3D)  # Rotation オブジェクトを作成
            euler_angles = rot.as_euler('xyz', degrees=True)  # XYZ順のオイラー角（度）
    #        print("Euler Angles (degrees):", euler_angles)

        # 結果の画面表示
            cv2.putText(frame, "Hit Esc key to terminate, v change calc mode, w toggle Intert", (10, 12),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            for i, corner in enumerate(corners):
                x, y = int(corner[0][0][0]), int(corner[0][0][1])  # 左上の座標
                marker_id = int(ids[i][0])
                cv2.putText(frame, f"ID: {marker_id}", (x, y - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                cv2.putText(frame, f"2D: {x},{y}", (10, 26),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
            
            cv2.putText(frame, f"3D: {marker_center_3D[0]:.1f},{marker_center_3D[1]:.1f},{marker_center_3D[2]:.1f} ", (10, 38),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        
    # 結果を表示
    cv2.imshow("Aruco Marker Detection", frame)
    cv2.imshow( "aeroTAP camera DepthMap", imgDepth )

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
