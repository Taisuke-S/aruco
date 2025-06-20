import cv2
import numpy as np
import aerotap
import time
# 移動平均
from collections import deque
# オイラー角への変換
from scipy.spatial.transform import Rotation

# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# cv2.aruco.estimatePoseSingleMarkers() で得られるマーカーコーナー2D座標から、
# カメラワールド座標3Dを求め、3Dコーナー座標から変換行列を求める
# 3D座標が求められない場合 = コーナーの距離座標が取得てきない場合 = 距離=0 は、無視
#
# 法線ベクトル (外積) を使って z 軸を求める
# 1つ目の点と2つ目の点のベクトル で x 軸を決める
# y 軸は外積 で求める
#
# 重要: 3Dカメラから得られる距離データ(Depth)にはブレが発生するため、移動平均化している
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

# 移動平均の管理
class DepthMovingAverage:
    def __init__(self, window_size):
        self.window_size = window_size
        self.values = deque()
        self.total = 0.0

    def update(self, value):
        if len(self.values) >= self.window_size:
            oldest = self.values.popleft()
            self.total -= oldest
        self.values.append(value)
        self.total += value

    def get_average(self):
        if not self.values:
            return 0.0  # Noneより0.0のほうが扱いやすい場合が多い
        return self.total / len(self.values)

# マーカーコーナーの各距離を移動平均から得る
def getDepthAve(no,corners,depth):
   global depthMoving_averages

   nDepthAve = np.zeros(4, dtype=np.float32)
   nDepth = getDepth(corners,depth)

   for i in range(depthMoving_averages):
       if nDepth[i]>0:
           depthMovingAverages[no][i].update(nDepth[i])
           nDepthAve[i] = depthMovingAverages[no][i].get_average()

#   print(f"nDepth {nDepthAve[0]:.0f}, {nDepthAve[1]:.0f}, {nDepthAve[2]:.0f}, {nDepthAve[3]:.0f}")
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

# 変換行列からオイラー角 (degree単位)を求める
def calcEulerAngle(T3D):

    # 1. 回転行列を取り出す（3×3）
    R_mat = T3D[:3, :3]
    # 2. Euler角に変換（Z-Y-X順、単位はrad）
    r = Rotation.from_matrix(R_mat)
    euler_angles = r.as_euler('zyx', degrees=True)
    return euler_angles

def drawreMarkerRect(image,corners,ids):
# 線の色と太さの指定
#    print("drawreMarkerRect called")
    line_color = (255, 255, 255)  # 赤 (BGR0形式)
    line_thickness = 4
    if corners is None or len(corners) == 0:
        print("No corners detected")
        return
# 各マーカーに対して枠を描画
    for i, corner in enumerate(corners):
        pts = corner[0].astype(int)  # 4点の座標 [4, 2]
        cv2.polylines(image, [pts], isClosed=True, color=line_color, thickness=line_thickness)
#        print(pts)

    # オプション：マーカーIDを表示
        if ids is not None:
            center = np.mean(pts, axis=0).astype(int)
            cv2.putText(image, f"ID {ids[i][0]}", tuple(center), cv2.FONT_HERSHEY_SIMPLEX,
                0.5, (0, 0, 255), 2, cv2.LINE_AA)

# マーカーclass
class Marker:

    base_3D_corners = None
    center_base  = None
    isValid = True

    marker_id =0
    corners = None
    rvec2D = None
    tvec2D = None
# コーナーの2D座標
    marker_corners_2D = np.array([
        [0,0,0],
        [0,0,0],
        [0,0,0],
        [0,0,0]
    ], dtype=np.int32)
# コーナーの3D座標
    marker_corners_3D = np.array([
        [0,0,0],
        [0,0,0],
        [0,0,0],
        [0,0,0]
    ], dtype=np.float32)
    center_marker = None
    center_marker3D = None

    rvec3D = None
    T3D = None
    T_T3D = None
    R3D = None
    t = None
    # コーナーの距離
    cornerDepth = np.array([0,0,0,0])
    euler_angles = None

    # def __init__(self,base_3D_corners,center_base):

    def setId(self,id):
       self.marker_id = id

    def processMaker(self,i,corners, rvecs2D, tvecs2D):

        selfisValid = False
        self.corners = corners
        self.rvec2D = rvecs2D[i]
        self.tvec2D = tvecs2D[i]

        # 回転ベクトルを回転行列に変換
        R, _ = cv2.Rodrigues(rvecs2D[i])

        # corners[0] の 4 点の座標 (2D)
        self.marker_corners_2D = corners[i].reshape(4, 2)  # (4,2) の形に変換
        # corners[0] の 4 点の距離(Z)
        self.cornerDepth = getDepthAve(i,self.marker_corners_2D,depth)
        # 距離が取得出来ていない場合
        if np.any(self.cornerDepth <= 0):
            return False, None, None

        # if Depth includes any valid 
        # if np.any(self.cornerDepth <= 0) or nViewMode ==1:
        #     if nViewMode ==0:
        #        cv2.putText(frame, "Invalid corners depth value!", (10, 38),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
        #     if nViewMode ==1:
        #          cv2.putText(frame, f"aruco:", (8, 18),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)

        #マーカーコーナー位置を3D座標に変換 
        # arucoの座標系に合わせるためにx軸の符号を反転？
        for i, point in enumerate(self.marker_corners_2D):
            x,y = point
            point3D = np.array([
            -(x - camera_center[0]) * self.cornerDepth[i] / nFocalLength[0],
            (y - camera_center[1]) * self.cornerDepth[i] / nFocalLength[1],            
            self.cornerDepth[i]
            ], dtype=np.float32)
    #              print(f"#{i} 3Dx {point3D[0]:.1f} = ({x}-{camera_center[0]}) * {cornerDepth[i] } / {nFocalLength[0]:.1f} ")
    #              print(f"#{i} 3Dy {point3D[1]:.1f} = ({y}-{camera_center[1]}) * {cornerDepth[i] } / {nFocalLength[1]:.1f} ")
            self.marker_corners_3D[i]  = point3D

        # マーカーの中心座標を計算
        self.center_marker3D = np.mean(self.marker_corners_3D, axis=0)

        # カメラからマーカーへの方向ベクトルを 3D カメラの座標系で求め、新しい回転行列を得る
        self.T3D = compute_transformation_matrix(self.marker_corners_3D)

        # マーカーらからの変換行列
        if nInvertMode ==1:
           self.T3D = invert_transformation_matrix(self.T3D)

        # 4x4 変換行列から回転行列（R）と並進ベクトル（t）の取得
        # 回転行列 (3x3)
        self.R3D = self.T3D[:3, :3]
        # 並進ベクトル (3x1)
        self.t = self.T3D[:3, 3]
        self.rvec3D, _ = cv2.Rodrigues(self.R3D)  # 3x3 → 3x1 の回転ベクトル

        # 回転行列 R からオイラー角（roll, pitch, yaw）を取得
        rot = Rotation.from_matrix(self.R3D)  # Rotation オブジェクトを作成
        self.euler_angles = rot.as_euler('xyz', degrees=True)  # XYZ順のオイラー角（度）

        self.isValid = True
        return True, self.R3D,self.t

    # 自身からmatrixへの相対変換
    def calcRelativeTransform(self,matrix):
#        print(matrix.shape) 
        self.T_T3D = matrix @ np.linalg.inv(self.T3D)
        return

    def drawMarker(self,nViewMode,nShowAxis):

        if not self.isValid:
            return

        offsetY = self.marker_id * 180
        # マーカーの座標系を描画
        if nViewMode ==0 and nShowAxis:
            cv2.drawFrameAxes(frame, camera_matrix, dist_coeffs, self.rvec3D, self.tvec2D, 0.03)

        if nViewMode ==1:
                cv2.putText(frame, f"aruco:", (8, 18),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)
        else:
                cv2.putText(frame, f"aeroTAP:", (8,18),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)

        center = self.marker_corners_2D.mean(axis=0)
        cx, cy= int(center[0]), int(center[1])  # 中央の座標
        # cv2.putText(frame, f"ID: {self.marker_id}", (cx, cy - 10),
        #             cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        # マーカーセンター位置2D
        cv2.putText(frame, f"Marker #{self.marker_id} Center 2D: {cx},{cy} pix", (10, 32+offsetY),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        # マーカーセンター位置3D
        cv2.putText(frame, f"Transform 3D: {self.center_marker3D[0]:.0f}, {self.center_marker3D[1]:.0f}, {self.center_marker3D[2]:.0f} mm",(10,62+offsetY),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)
        cv2.putText(frame, f"Transform 3D: {self.center_marker3D[0]:.0f}, {self.center_marker3D[1]:.0f}, {self.center_marker3D[2]:.0f} mm",(10,62+offsetY),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        # オイラー角 (degree単位)
        cv2.putText(frame, f"euler_angles: roll {self.euler_angles[0]:.2f}, pitch {self.euler_angles[1]:.2f}, yaw {self.euler_angles[2]:.2f} degree",(10,80+offsetY), cv2.FONT_HERSHEY_SIMPLEX,0.5,(0,0,0),2)
        cv2.putText(frame, f"euler_angles: roll {self.euler_angles[0]:.2f}, pitch {self.euler_angles[1]:.2f}, yaw {self.euler_angles[2]:.2f} degree",(10,80+offsetY), cv2.FONT_HERSHEY_SIMPLEX,0.5,(255,255,255),1)

        # マーカーコーナー位置3D　mm
        for i, corner3D in enumerate(self.marker_corners_3D):
                x, y,z = corner3D
                cv2.putText(frame, f"{i}:{x:.1f},{y:.1f},{z:.1f}", (10, 100+i*20+offsetY),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        return

    def drawrelativeAxis(self):
        R = self.T_T3D[:3, :3]          # 回転行列 (3x3)
        # 回転行列を回転ベクトルに変換
        rvec, _ = cv2.Rodrigues(R)
        # マーカー位置に相対座標系を描画
        cv2.drawFrameAxes(frame, camera_matrix, dist_coeffs, rvec, self.tvec2D, 0.03)

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
cameraWidth = 1280
cameraHeight = 720
# カメラ内部を90として、30FPSで処理するには  nFPS = 190 とする
nFPS = 30
# ArUcoマーカーの辞書を定義 (DICT_4X4_50 など他の辞書も使用可)
marker_size = 0.05  # 5cmのマーカーと仮定
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
aruco_params = cv2.aruco.DetectorParameters()

# カメラを起動
if not aerotap.getCamModeAndType():
     raise ValueError("Camera detection Error")
# カメラがUSB3.0接続されていることを確認
nIsUSB30 = aerotap.IsUSB30()
if nIsUSB30 ==1:
     raise ValueError("Camera is connected with USB2.0 mode, Please check the cable and connection")

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

point3D = np.array([0,0,0], dtype=np.float32)

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
depthMovingAverages = [
 [DepthMovingAverage(depthAve_size) for _ in range(depthMoving_averages)],
 [DepthMovingAverage(depthAve_size) for _ in range(depthMoving_averages)]
]

# マーカーリストの定義 
markers = [
    Marker(),
    Marker()
]

# マーカーの中心座標を計算(カメラの中心位置)
center_base = np.mean(base_3D_corners, axis=0)

# nViewMode with aeroTAP or aruco
nViewMode =0
# nInvertMode View from Camera or Marker
nInvertMode =0
# nRelative relative from marker 0 or marker 1
nRelative =0
# nShowInfo Show/Hide information
nShowInfo =1

# claheライブラリ準備
clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))

# カメラメイン ループ
while True:
  if aerotap.IsNewFrame():

    frame = aerotap.ReadImage(7)
    depth = aerotap.ReadDepth()
    # convert depth map to bgr ColorImage
    if( not (depth is None) ):
       imgDepth = aerotap.DepthToRGB(depth)
    # clage 返還後のグレー画像の確認用
#       cv2.imshow( "aeroTAP camera DepthMap", imgDepth )

    # グレースケールに変換
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
   # CLAHEで前処理
    gray = clahe.apply(gray)

    # マーカーを検出
    detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)
    corners, ids, _ = detector.detectMarkers(gray)

    # リセット マーカー
    markers[0].isValid = markers[1].isValid = False
    # マーカーが見つかった場合
    if ids is not None:
        
        rvecs2D, tvecs2D, _ = cv2.aruco.estimatePoseSingleMarkers(corners, marker_size, camera_matrix, dist_coeffs)

        drawreMarkerRect(imgDepth,corners,ids)
        cv2.aruco.drawDetectedMarkers(frame, corners, ids)
        for i, rvec in enumerate(rvecs2D):

            # マーカーの座標系を描画
            if nViewMode ==1:
               cv2.drawFrameAxes(frame, camera_matrix, dist_coeffs, rvec, tvecs2D[i],0.03)

            marker_id = int(ids[i][0])
            # Valid Maker ID?
            if marker_id >1:
              continue
            if i >2:
              break
            markers[i].setId(marker_id)
            success, R3D,t = markers[i].processMaker(i,corners, rvecs2D, tvecs2D)
            if not success:
              continue

            if nShowInfo ==1:
                markers[i].drawMarker(nViewMode,False if nRelative < 2 else True)

    # 2つのマーカーが見つかった場合のみ
    if markers[0].isValid and markers[1].isValid: 
    # 0から1への相対変換
        markers[0].calcRelativeTransform(markers[1].T3D)
    # 1から0への相対変換
        markers[1].calcRelativeTransform(markers[0].T3D)

        euler_angles = None
        if nRelative == 0:
            euler_angles = calcEulerAngle(markers[0].T_T3D)
        else:
            euler_angles = calcEulerAngle(markers[1].T_T3D)
        # オイラー角 (degree単位)
        offsetY = 280
        offsetX = 380
        translation = None

        if nRelative == 0:
            markers[0].drawrelativeAxis()
            # 平行移動（translation vector）の取り出し
            translation = markers[0].T_T3D[:3, 3]
            cv2.putText(frame, f"Relative #0 euler_angles:",(cameraWidth-offsetX,offsetY), cv2.FONT_HERSHEY_SIMPLEX,0.5,(0,0,0),2)
            cv2.putText(frame, f"Relative #0 euler_angles:",(cameraWidth-offsetX,offsetY), cv2.FONT_HERSHEY_SIMPLEX,0.5,(255,255,255),1)
            print("Relative #0 euler_angles",f"r: {euler_angles[0]:.2f}, p: {euler_angles[1]:.2f}, y: {euler_angles[2]:.2f}")
        elif nRelative == 1:
            markers[1].drawrelativeAxis()
            # 平行移動（translation vector）の取り出し
            translation = markers[1].T_T3D[:3, 3]
            cv2.putText(frame, f"Relative #1 euler_angles:",(cameraWidth-offsetX,offsetY), cv2.FONT_HERSHEY_SIMPLEX,0.5,(0,0,0),2)
            cv2.putText(frame, f"Relative #1 euler_angles:",(cameraWidth-offsetX,offsetY), cv2.FONT_HERSHEY_SIMPLEX,0.5,(255,255,255),1)
            print("Relative #1 euler_angles",f"r: {euler_angles[0]:.2f}, p: {euler_angles[1]:.2f}, y: {euler_angles[2]:.2f}")

        cv2.putText(frame,f"Translation x: {translation[0]:.2f},y: {translation[1]:.2f},z: {translation[2]:.2f} mm",(cameraWidth - offsetX, offsetY + 20 ),cv2.FONT_HERSHEY_SIMPLEX,0.5, (0, 0, 0), 2)
        cv2.putText(frame,f"Translation x: {translation[0]:.2f},y: {translation[1]:.2f},z: {translation[2]:.2f} mm",(cameraWidth - offsetX, offsetY + 20 ),cv2.FONT_HERSHEY_SIMPLEX,0.5, (255, 255, 255), 1)

        labels = ["roll", "pitch", "yaw"]
        for i, label in enumerate(labels):
            cv2.putText(frame,f"{label} {euler_angles[i]:.2f} degree",(cameraWidth - offsetX, offsetY + 20 * (i + 2)),cv2.FONT_HERSHEY_SIMPLEX,0.5, (0, 0, 0), 2)
            cv2.putText(frame,f"{label} {euler_angles[i]:.2f} degree",(cameraWidth - offsetX, offsetY + 20 * (i + 2)),cv2.FONT_HERSHEY_SIMPLEX,0.5, (255, 255, 255), 1)

    # 情報を表示
    cv2.putText(frame, "Hit Esc key to terminate, v change calc mode, w toggle Invert, r toggle relative, s show/hide information", (10, cameraHeight-20),cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
    # Title表示
    cv2.imshow("Aruco Multi-Marker Detection with SVD", frame)
    if( not (depth is None) ):
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
    if key & 0xFF == ord("r"):
            nRelative += 1
            nRelative = nRelative % 3
    if key & 0xFF == ord("s"):
        if nShowInfo == 0:
            nShowInfo =1
        else:
            nShowInfo =0
  else:
    err = aerotap.GetLastError()
#   ant camera error?
    if  err:
        break
  time.sleep(0.033)

#cap.release()
aerotap.Close()

cv2.destroyAllWindows()
