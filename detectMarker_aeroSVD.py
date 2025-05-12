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
# カメラワールド座標3Dを求め、基準４コーナーからの変換行列をSVDで求める
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

# 変換行列からオイラー角 (degree単位)を求める
def calcEulerAngle(T3D):

    # 1. 回転行列を取り出す（3×3）
    R_mat = T3D[:3, :3]
    # 2. Euler角に変換（Z-Y-X順、単位はrad）
    r = Rotation.from_matrix(R_mat)
    euler_angles = r.as_euler('zyx', degrees=True)
    return euler_angles

# マーカーclass
class Marker:

    base_3D_corners = None
    center_base  = None
    isValid = True

    marker_id =0
    corners = None
    rvec2D = None
    tvec2D = None
    marker_corners_2D = None
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
        R, _ = cv2.Rodrigues(rvec)

        # マーカーの座標系を描画 aruco 姿勢
        if nViewMode ==1:
            cv2.drawFrameAxes(frame, camera_matrix, dist_coeffs, rvec, self.tvec2D, 0.03)

    #        print("cornsers: ", corners) 
        self.marker_corners_2D = corners[i].reshape(4, 2) 
        self.center_marker = np.mean(self.marker_corners_2D, axis=1)
    #        print("center_marker: ", center_marker) 
        #マーカー位置の距離 
        self.cornerDepth = getDepthAve(self.marker_corners_2D,depth)
        # 距離が取得出来ていない場合
        if np.any(self.cornerDepth <= 0):
            return False, None, None

    #        point3D = np.zeros(3, dtype=np.float32)
        #マーカーコーナー位置を3D座標に変換 
        # arucoの座標系に合わせるためにx軸の符号を反転？
        corners_array = corners[i]
        for j, point in enumerate(corners_array[0]):
            x,y = point
            point3D = np.array([
            -(x - camera_center[0]) * self.cornerDepth[j] / nFocalLength[0],
            (y - camera_center[1]) * self.cornerDepth[j] / nFocalLength[1],
            self.cornerDepth[j]
            ], dtype=np.float32)
            self.marker_corners_3D[j]  = point3D            

        # マーカーの中心座標を計算
        self.center_marker3D = np.mean(self.marker_corners_3D, axis=0)

        # 座標系を中心からの相対的な位置に変換
        P1 = self.marker_corners_3D - self.center_marker3D  # マーカーの相対座標系
        P2 = base_3D_corners - center_base  # 基準座標系の相対座標系

        # 3. SVDを使用して回転行列を求める
        # H 行列を計算 (P2.T * P1)
        H = np.dot(P2.T, P1)

        # SVDを適用
        U, S, Vt = np.linalg.svd(H)

        # 回転行列 R の計算
        self.R3D = np.dot(Vt.T, U.T)
        # 行列式をチェックして反射行列でないか確認
        det_R = np.linalg.det(self.R3D)
    #       print("行列式 det(R):", det_R)

        # もし行列式が -1 なら、反射行列なので修正します
        if det_R < 0:
            Vt[-1, :] *= -1
            self.R3D = np.dot(Vt.T, U.T)

        # 並進ベクトルを計算
        self.t = self.center_marker3D - R @ center_base

        # マーカーの中心座標を計算
        self.center_marker3D = np.mean(self.marker_corners_3D, axis=0)

        # 4×4 の同次変換行列を作成
        self.T3D = np.eye(4)  # 単位行列 (4×4)
        self.T3D[:3, :3] = self.R3D  # 左上に R を配置
        self.T3D[:3, 3] = self.t   # 並進ベクトル t を配置

         # マーカーらからの変換行列
        if nInvertMode ==1:
           self.T3D = invert_transformation_matrix(self.T3D)

        self.rvec3D, _ = cv2.Rodrigues(self.R3D)  # 3x3 → 3x1 の回転ベクトル

        # 回転ベクトルからオイラー角（roll, pitch, yaw）に変換 (ZYX順)
        rotation = Rotation.from_rotvec(self.rvec3D.flatten()) 
        self.euler_angles = rotation.as_euler('zyx', degrees=True)  # ZYX順のオイラー角 (degree単位)

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
        # マーカーの座標系を描画 aeroTAP姿勢
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


# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# メイン処理
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#
cameraWidth = 640
cameraHeight = 480
# カメラ内部を90として、30FPSで処理するには  nFPS = 190 とする
nFPS = 30
# Arucoマーカーの辞書を定義 (DICT_4X4_50 など他の辞書も使用可)
marker_size = 0.05  # 5cmのマーカーと仮定
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
aruco_params = cv2.aruco.DetectorParameters()

# aeroTAP カメラを起動
if not aerotap.getCamModeAndType():
     raise ValueError("Camera detection Error")
# カメラがUSB3.0接続されていることを確認
nIsUSB30 = aerotap.IsUSB30()
if nIsUSB30 ==1:
     raise ValueError("Camera is connected with USB2.0 mode, Please check the cable and connection")

# Initialize camera check
if not aerotap.Init(0,nFPS,aerotap.camMode):
     raise ValueError("Error aeroInit")
   
#aerotap.EnableDepthFilter(bEnableDepthFilter)
if not aerotap.StartCam(cameraWidth,cameraHeight):
     raise ValueError("Camera Start Error")

print("Starting camera...type 'Esc' to terminate.")      

nFocalLength = np.array(
        [590.00,590.00]
    , dtype=np.float32)

nFocalLength[0], nFocalLength[1] = aerotap.GetFocalLength(0)
print(f"Focal Length: w={nFocalLength[0]:.3f}, h={nFocalLength[1]:.3f}")

# カメラ中心位置
camera_center = (cameraWidth/2, cameraHeight / 2)

point3D = np.array([0,0,0], dtype=np.float32)

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

# nViewMode with aeroTAP or aruco
nViewMode =0
# nInvertMode View from Camera or Marker
nInvertMode =0
# nRelative relative from marker 0 or marker 1
nRelative =0
# nShowInfo Show/Hide information
nShowInfo =1

# マーカーリストの定義 
markers = [
    Marker(),
    Marker()
]
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
       cv2.imshow( "aeroTAP camera DepthMap", imgDepth )

    aerotap.UpdateFrame()

    # グレースケールに変換
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    # CLAHEで前処理
    gray = clahe.apply(gray)
    # clage 返還後のグレー画像の確認用
#    cv2.imshow( "aeroTAP clahe", gray )

    # マーカーを検出
    detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)
    corners, ids, _ = detector.detectMarkers(gray)

    # リセット マーカー
    markers[0].isValid = markers[1].isValid = False
    # マーカーが見つかった場合
    if ids is not None:
        # すべての見つかったマーカー枠、マーカーID 描画
        cv2.aruco.drawDetectedMarkers(frame, corners, ids)
        rvecs2D, tvecs2D, _ = cv2.aruco.estimatePoseSingleMarkers(corners, marker_size, camera_matrix, dist_coeffs)

        # マーカー毎にMaker classを構成
        for i, rvec in enumerate(rvecs2D):
            marker_id = int(ids[i][0])
            # Valid Maker ID?  should be 0 or 1
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

        # オイラー角情報の表示 (degree単位)
        offsetY = 240
        offsetX = 200

        if nRelative == 0:
            markers[0].drawrelativeAxis()
            cv2.putText(frame, f"Relative #0 euler_angles:",(cameraWidth-offsetX,offsetY), cv2.FONT_HERSHEY_SIMPLEX,0.5,(0,0,0),2)
            cv2.putText(frame, f"Relative #0 euler_angles:",(cameraWidth-offsetX,offsetY), cv2.FONT_HERSHEY_SIMPLEX,0.5,(255,255,255),1)
            print("Relative #0 euler_angles",f"r: {euler_angles[0]:.2f}, p: {euler_angles[1]:.2f}, y: {euler_angles[2]:.2f}")
        elif nRelative == 1:
            markers[1].drawrelativeAxis()
            cv2.putText(frame, f"Relative #1 euler_angles:",(cameraWidth-offsetX,offsetY), cv2.FONT_HERSHEY_SIMPLEX,0.5,(0,0,0),2)
            cv2.putText(frame, f"Relative #1 euler_angles:",(cameraWidth-offsetX,offsetY), cv2.FONT_HERSHEY_SIMPLEX,0.5,(255,255,255),1)
            print("Relative #1 euler_angles",f"r: {euler_angles[0]:.2f}, p: {euler_angles[1]:.2f}, y: {euler_angles[2]:.2f}")

        labels = ["roll", "pitch", "yaw"]
        for i, label in enumerate(labels):
            cv2.putText(frame,f"{label} {euler_angles[i]:.2f} degree",(cameraWidth - offsetX, offsetY + 20 * (i + 1)),cv2.FONT_HERSHEY_SIMPLEX,0.5, (0, 0, 0), 2)
            cv2.putText(frame,f"{label} {euler_angles[i]:.2f} degree",(cameraWidth - offsetX, offsetY + 20 * (i + 1)),cv2.FONT_HERSHEY_SIMPLEX,0.5, (255, 255, 255), 1)


    # 情報を表示
    cv2.putText(frame, "Hit Esc key to terminate, v change calc mode, w toggle Invert, r toggle relative, s show/hide information", (10, cameraHeight-20),cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
     # Title表示
    cv2.imshow("Aruco Multi-Marker Detection with 3D", frame)

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
