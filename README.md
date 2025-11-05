# aruco2
 aruco for aeroTAP

#
#__author__ = "nextEDGE Technology"
#__copyright__ = "Copyright (C) 2025 nextEDGE Technology K.K."
#__license__ = "Public Domain"
#__version__ = "2.4"
#
# last updated:2025-05-12

# Changes
1. Defined a Marker class to manage multiple (2) ArUco markers, allowing each marker to store its own pose matrix.
2. Calculated the relative pose matrix between pose matrices managed per marker, and displayed roll, pitch, and yaw (in degrees).
3. Enabled the Marker class to handle the display of information for each marker.
4. Implemented CLAHE as a preprocessing step to improve marker detection accuracy and robustness.
5. Added a display mode toggle with the 's' key to show/hide marker-specific information.
6. Enabled switching between relative poses from Marker ID 1 and Marker ID 2 using the 'r' key.
7. maker.py generates ArUco marker IDs 0 and 1.
8. Updated aeroTAP_CAM.dll:
   Now supports saving and loading the FocalLength value directly within the camera.
   Previously, a default standard FocalLength value was used, but now it can be set and handled as a camera-specific value.
   As a result, more accurate 3D data (X, Y coordinate values) can be obtained

# Prerequisites
1. Run VSCode
2. python -m venv aruco
3. Choose Command Palette -> Select Python interpreter -> venv aruco

pip install opencv-python opencv-contrib-python numpy
pip install --upgrade scipy

>>> print(np.__version__) 
1.24.1pi
>>> print(scipy.__version__)
1.15.2

# Samples
aerotap.py                     aeroTAP SDK for python wrapper
detectMarker.py                aruco sample with using Web cam
detectMarker_aerotap.py        aruco sample with using aeroTAP 3D USB Camera
detectMarker_aeroSVD.py        A sample for obtaining the marker transformation matrix using SVD from four corners instead of Aruco
　　　　　　　　　　　　　　　　　*stable ?
detectMarker_aerotap3D.py      A sample for converting the four ArUco corners to 3D and obtaining the marker transformation matrix.
detectMarker_aerotapDEBUG.py   DEBUG version will save RAW data to ./RAW folder
detectMarker_aerotap3DDEBUG.py DEBUG version will load from RAW data and process aruco marker

marker.py                      Marker output script
aruco_marker0.png              Marker ID 0 file
aruco_marker1.png              Marker ID 1 file

aeroTAP_CAM.dll                aeroTAP SDK Library DLL
aeroTAP_CAMMP.dll              aeroTAP SDK Library DLL OpenMP enabled
eSPDI_DM.dll                   aeroTAP 3D USB Camera low level Library

ZDColorPalette.py              aerotap.py　Supplementary: A library for generating color lookup for depth maps in Python.

# Algorithm - detectMarker_aeroSVD.py
 From the 2D marker corner coordinates obtained using cv2.aruco.estimatePoseSingleMarkers(),
 compute the 3D camera world coordinates and derive the transformation matrix from the 3D corner coordinates using SVD.
 Set standard marker coordinates as a reference and compute the transformation matrix 
 that relates them to the measured marker coordinates.
 If 3D coordinates cannot be obtained (i.e., corner distance coordinates cannot be retrieved or distance = 0), ignore them.

 Important: Since the depth data obtained from the 3D camera has fluctuations,
 a moving average is applied to smooth the values.


# Algorithm - detectMarker_aerotap3D.py
 From the 2D marker corner coordinates obtained using cv2.aruco.estimatePoseSingleMarkers(),
 compute the 3D camera world coordinates and derive the transformation matrix from the 3D corner coordinates.
 If 3D coordinates cannot be obtained (i.e., corner distance coordinates cannot be retrieved or distance = 0), ignore them.

 Determine the z-axis using the normal vector (cross product).
 Define the x-axis using the vector between the first and second points.
 Compute the y-axis using the cross product.

 Important: Since the depth data obtained from the 3D camera has fluctuations,
 a moving average is applied to smooth the values.

# Operation Method  
 Press the 'v' key to switch ViewMode:  
      Toggle between estimation using Aruco and estimation using 3D coordinates.  
 Press the 's' key to switch Hide/Show detected Marker Information
 Press the 'w' key to calc Invert deformation matrix from Maker or camera
 Press the 'r' key to calc Relative from Maker 0 , Maker 1, or no relative 

# Output
  transform from camera center x,y,z ( mm )
  euler_angles roll, pitch , yaw (degree)

# History
1.6  Modified view data
     Showing transforn 3D and euler_angles (roll, pitch, yaw) 
2.0  Modified to support multiple markers 
2.2  Added DEBUG samples to confirm Marker depth is detected correctly
     detectMarker_aerotapDEBUG.py  
     detectMarker_aerotap3DDEBUG.py
     added drawreMarkerRect() to draw marker rect in Depth Map image
2.4  fixed issue with getDepthAve calculating incorrect depth average
     added to show transaltion between markers
2.5  Fixed incorrect depth value with aeroTAP 3D USB G2 camera
     use camMode =11 rather than camMode =12 to correct
