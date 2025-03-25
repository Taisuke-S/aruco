#
#__author__ = "nextEDGE Technology"
#__copyright__ = "Copyright (C) 2025 nextEDGE Technology K.K."
#__license__ = "Public Domain"
#__version__ = "1.4"
#
# last updated:2025-03-25

# 環境設定
pip install opencv-python opencv-contrib-python numpy
pip install --upgrade scipy

#サンプルプログラム リスト
aerotap.py                     aeroTAP SDK for python wrapper
detectMarker.py                Webカメラを使ったarucoサンプル
detectMarker_aerotap.py        aeroTAP 3D USBカメラを使ったarcoサンプル
detectMarker_aeroSVD.py        arucoの代わりに4コーナーからSVDでマーカー変換行列を求めるサンプル
　　　　　　　　　　　　　　　　　こちらの方が安定している?
detectMarker_aerotap3D.py      arucoの4コーナーを3D化し、マーカー変換行列を求めるサンプル

marker.py                      マーカー画像出力プログラム
aruco_marker.png               マーカー画像データ

aeroTAP_CAM.dll                aeroTAP SDK Library DLL
aeroTAP_CAMMP.dll              aeroTAP SDK Library DLL OpenMP enabled版
eSPDI_DM.dll                   aeroTAP 3D USB カメラ low levelライブラリ

ZDColorPalette.py              aerotap.py　補足py depthmapのカラーLookup生成ライブラリ

# アルゴリズム - detectMarker_aeroSVD.py
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# cv2.aruco.estimatePoseSingleMarkers() で得られるマーカーコーナー2D座標から、
# カメラワールド座標3Dを求め、3Dコーナー座標から変換行列をSVDから求める
# 標準マーカー座標を仮に設定し、実測マーカーとの関係変換行列を求める
# 3D座標が求められない場合 = コーナーの距離座標が取得てきない場合 = 距離=0 は、無視
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
# v キーでViewModeを切り替え  arucoで求める <-> 3Dから求める 