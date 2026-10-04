RealSense D436 visual model and nominal simulation cameras

Source: https://github.com/realsenseai/realsense-ros
Revision: 9a11121700cb4780e273e34141f6402fe184321d
Official D436 description: realsense2_description/urdf/_d436.urdf.xacro
The official D436 xacro references meshes/d435.dae for the aluminum case.
Copyright 2026 RealSense, Inc. All Rights Reserved (D436 description).
Apache-2.0; see LICENSE. Source and converted-file hashes are in source.json.

Modifications: DAE triangles converted to binary STL in meters. Bake the xacro
visual rotation RPY=(pi/2,0,pi/2) and translation=(0.0043,-0.0175,0) into vertices
so these meshes use camera_link coordinates (+X forward, +Y left, +Z up).
Remove zero-area triangles; split at 150000 faces to fit MuJoCo's per-STL limit.
No mesh simplification. CAD diffuse gray retained as the MJCF material.
The camera bracket is a project-specific visual approximation. Housing and
bracket have no contact or dynamic mass contribution.

Installation in models/mjcf/aviator.xml (world meters, fixed to aircraft):
- Base plate follows the common arm mounting bracket's 10-degree slope.
  Bracket top at X=-0.91333 is Z=0.20338802; plate thickness is 10 mm.
- Bottom screw frame: (-0.91333, 0, 0.320).
- Orientation: yaw -2.7501 degrees, pitch +18.5472 degrees (downward optical aim).
- camera_link from bottom screw: (0.0106, 0.0175, 0.0125), per official xacro.
- RGB origin from camera_link: (-0.00555, 0.015, 0); depth origin: (0,0,0).
- MJCF cameras look along local -Z: xyaxes="0 -1 0 0 0 1" converts ROS axes.
- Aim solved at roll=0, pitch slide=-0.085 toward yoke_apriltag_center.
- Optical range is about 0.70 m at this neutral position.

Official nominal specs: https://www.realsenseai.com/products/d436/ (Tech Specs)
- Case: 90 x 25 x 25 mm (xacro depth is 25.05 mm).
- RGB: 1280 x 800, horizontal/vertical FOV 90/65 degrees, maximum 60 fps.
- Depth: 1280 x 720, horizontal/vertical FOV 87/58 degrees, maximum 90 fps.
- Depth minimum 0.2 m, ideal range 0.3 to 3 m.
The simulation capture loop remains 30 Hz. Maxima are not simultaneous stream
mode guarantees. The main Tech Specs table is used for RGB FOV; the product
page FAQ lists a different RGB FOV.

MJCF cockpit_apriltag is the RGB camera used by nodes/simulation/camera.cpp;
realsense_d436_depth is an additional ideal depth viewpoint, not a published
active-stereo depth stream. Intrinsics derive from fx=W/(2*tan(HFOV/2)) and
fy=H/(2*tan(VFOV/2)), centered principal point. RGB K: fx=640, fy=627.87423,
cx=640, cy=400. sensorsize uses normalized units with the proper aspect ratio;
focal/sensorsize ratios encode these nominal intrinsics, not a physical sensor
size or factory calibration. No lens distortion or sensor noise is modeled.

CAD housing uses geom group 1, excluded from its own sensor rendering because
the nominal optical centers sit behind opaque CAD glass. Viewer renders it.
Custom Python renders should pass a MjvOption with geomgroup[1]=0. Other scene
geometry remains visible. tests/mjcf_apriltag_test.py renders and detects ID 0
at 33 wheel poses, checks full board visibility and tag-center displacement.
