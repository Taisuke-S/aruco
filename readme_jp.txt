#
#__author__ = "nextEDGE Technology"
#__copyright__ = "Copyright (C) 2025 nextEDGE Technology K.K."
#__license__ = "Public Domain"
#__version__ = "2.0"
#
# last updated:2025-05-12

# 変更箇所
1. 複数(2個)のarucoマーカーを検出管理するために Marker class を定義し、マーカー毎に姿勢行列を保持できるようにした。   
2. マーカー毎に管理された姿勢行列間での相対姿勢行列を求め、roll, pitch , yaw (degree) 値を表示
3. 各マーカーの情報表示もMarker classで処理できるようにした
4. マーカーの検出精度と検出精度を上げるために前処理としてCLAHE処理を実装した
5. 表示モード s keyでマーカー毎の情報の表示/非表示の追加
6. r Keyで相対姿勢行列をMarker ID 1からの相対姿勢、ID 2からの相対姿勢を切り替える
7. maker.py arucoマーカーID 0,1を生成
8. aeroTAP_CAM.dll 最新版
     FocalLength情報をカメラ内部に保存し、読み込む機能のサポート、これにより
     従来、標準値としてのFocalLength値を使っていたが、カメラ固有の値として設定し、扱うことが可能
     結果、より精度の高い 3Dデータ ( X,Y座標値 ) が得られる
9. カメラ起動時にUSB接続モードをチェック、USB3.0で接続されていない場合、強制終了

# 環境設定
1. VSCode起動
2. python -m venv aruco
3. Command Palette -> Select Python interpreter -> venv aruco を選択

pip install opencv-python opencv-contrib-python numpy
pip install --upgrade scipy


#サンプルプログラム リスト
aerotap.py                     aeroTAP SDK for Python wrapper
detectMarker.py                Webカメラを使ったarucoサンプル (複数マーカー対応)
detectMarker_aerotap.py        aeroTAP 3D USBカメラを使ったarucoサンプル (複数マーカー対応)
detectMarker_aeroSVD.py        arucoの代わりにマーカー4コーナー(x,y,z)と基準4コーナー点の関係をSVDで変換行列を求めるサンプル
　　　　　　　　　　　　　　　　　こちらの方が安定している?
detectMarker_aerotap3D.py      arucoの4コーナーを3D化し、マーカー変換行列を求めるサンプル
detectMarker_aerotapDEBUG.py   DEBUG 版は、RAWデータを./RAWフォルダーに保存します
detectMarker_aerotap3DDEBUG.py DEBUG 版は、カメラの代わりにRAWフォルダーからのデータを処理します。

marker.py                      マーカー画像出力プログラム id 0,  id 1
aruco_marker0.png              マーカー id 0 画像データ
aruco_marker1.png              マーカー id 1 画像データ

aeroTAP_CAM.dll                aeroTAP SDK Library DLL
aeroTAP_CAMMP.dll              aeroTAP SDK Library DLL OpenMP enabled版
eSPDI_DM.dll                   aeroTAP 3D USB カメラ low levelライブラリ

ZDColorPalette.py              aerotap.py　補足py depthmapのカラーLookup生成ライブラリ

# アルゴリズム - detectMarker_aeroSVD.py
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# cv2.aruco.estimatePoseSingleMarkers() で得られるマーカーコーナー2D座標から、
# カメラワールド座標3Dを求め標準マーカー座標を仮に設定し、実測マーカーとの関係をSVDで変換行列
として求める
# 3D座標が求められない条件 = コーナーの距離座標が取得てきない場合 = 距離=0 は、無視
#
# 重要: 3Dカメラから得られる距離データ(Depth)にはブレが発生するため、移動平均化している
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


# アルゴリズム - detectMarker_aerotap3D.py
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

# 操作方法
 v キーでViewModeを切り替え  arucoで求める <-> 3Dから求める 
 s キーでマーカー情報の表示On/Off切り替え
 w キーでカメラからの姿勢行列からマーカーからの姿勢行列に計算切り替え
 r キーで相対姿勢行列をMaker ID 0からの相対位置、ID 1からの相対値、相対位置計算なしに切り替え
   座標系の表示は、0 の時、マーカー ID 1 上に表示 ( ID 0 との相対姿勢 )
   座標系の表示は、1 の時、マーカー ID 0 上に表示 ( ID 1 との相対姿勢 )
   座標系の表示は、2 の時、マーカー ID 0，1 それぞれカメラ空間、またはマーカー空間からの姿勢を表示
 
# 出力結果
  transform 中央からの位置 x,y,z ( mm )
  euler_angles roll, pitch , yaw (degree)

# History
1.6  Modified view data
     Showing transforn 3D and euler_angles (roll, pitch, yaw) 
2.0  Modified to support multiple markers 
2.2  Added DEBUG samples to confirm Marker depth is detected correctly
     detectMarker_aerotapDEBUG.py  
     detectMarker_aerotap3DDEBUG.py
     added drawreMarkerRect() to draw marker rect in Depth Map image
2.4  マーカーのDepth計算を移動平均で求めていたが、マーカー毎の移動平均ではなく、2つのマーカーの移動平均になっていた問題
     マーカー間の平行移動(Translation)の表示