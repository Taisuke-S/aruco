# aruco
 aruco for aeroTAP

#
#__author__ = "nextEDGE Technology"
#__copyright__ = "Copyright (C) 2025 nextEDGE Technology K.K."
#__license__ = "Public Domain"
#__version__ = "1.4"
#
# last updated:2025-03-25

# Prerequisites
pip install opencv-python opencv-contrib-python numpy
pip install --upgrade scipy

# Samples
aerotap.py                     aeroTAP SDK for python wrapper
detectMarker.py                aruco sample with using Web cam
detectMarker_aerotap.py        aruco sample with using aeroTAP 3D USB Camera
detectMarker_aeroSVD.py        A sample for obtaining the marker transformation matrix using SVD from four corners instead of Aruco
　　　　　　　　　　　　　　　　　*stable ?
detectMarker_aerotap3D.py      A sample for converting the four ArUco corners to 3D and obtaining the marker transformation matrix.
marker.py                      Marker output script
aruco_marker.png               Marker file

aeroTAP_CAM.dll                aeroTAP SDK Library DLL
aeroTAP_CAMMP.dll              aeroTAP SDK Library DLL OpenMP enabled
eSPDI_DM.dll                   aeroTAP 3D USB Camera low level Library

ZDColorPalette.py              aerotap.py　Supplementary: A library for generating color lookup for depth maps in Python.

Algorithm - detectMarker_aeroSVD.py
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# From the 2D marker corner coordinates obtained using cv2.aruco.estimatePoseSingleMarkers(),
# compute the 3D camera world coordinates and derive the transformation matrix from the 3D corner coordinates using SVD.
# Set standard marker coordinates as a reference and compute the transformation matrix 
# that relates them to the measured marker coordinates.
# If 3D coordinates cannot be obtained (i.e., corner distance coordinates cannot be retrieved or distance = 0), ignore them.
#
# Important: Since the depth data obtained from the 3D camera has fluctuations,
# a moving average is applied to smooth the values.
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


# Algorithm - detectMarker_aerotap3D.py
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# From the 2D marker corner coordinates obtained using cv2.aruco.estimatePoseSingleMarkers(),
# compute the 3D camera world coordinates and derive the transformation matrix from the 3D corner coordinates.
# If 3D coordinates cannot be obtained (i.e., corner distance coordinates cannot be retrieved or distance = 0), ignore them.
#
# Determine the z-axis using the normal vector (cross product).
# Define the x-axis using the vector between the first and second points.
# Compute the y-axis using the cross product.
#
# Important: Since the depth data obtained from the 3D camera has fluctuations,
# a moving average is applied to smooth the values.
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

# Operation Method  
# Press the 'v' key to switch ViewMode:  
# Toggle between estimation using Aruco and estimation using 3D coordinates.  
