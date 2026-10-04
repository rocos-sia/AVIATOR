apriltag_36h11_id0.png
Source: docs/Apriltag80id0.pdf (tag36h11, ID 0, black border size 80 mm).
Extracted the embedded 10x10 RGB image using Poppler pdfimages -png (image 002),
then enlarged to 640x640 with nearest-neighbor sampling. No marker bits changed.
The outer one-cell white margin is retained: a 100 mm board gives an 80 mm tag.
MuJoCo material uses one 2D texture repeat and no specular reflection.

Mount: models/mjcf/aviator.xml, steering_wheel child geom yoke_apriltag.
Local +Z is the shaft axis; local +Y is up at neutral roll. Board center is
(0, 0.106, 0.320) m, in the central hub plane and 106 mm above the shaft.
Front-center site yoke_apriltag_center = (0, 0.106, 0.3205) m. No cylindrical
standoff is used. The 100 mm board's lower edge is at Y=0.056 m, approximately
3 mm above the central housing (Y~0.0526 m within the board's width).
An upward-offset scan first cleared the pilot camera near 98 mm; 106 mm also
provides geometric clearance of the housing rather than relying on perspective.
The board adds no mass or collision contacts. Both roll_input_joint and
pitch_input_joint move the tag, including its offset from the rotation axis.
Visibility is checked from cockpit_apriltag, not guaranteed from arbitrary views.

Fixed MJCF camera cockpit_apriltag faces the tag from the pilot side. The simulation
camera uses it when present and retains the legacy view for other models.
The existing simulation camera.detection messages still use model ground truth;
adding a tag does not switch that protocol to an image-based AprilTag detector.
The actual PDF texture is independently decoded from rendered images in the test:
  MUJOCO_GL=egl python3 tests/mjcf_apriltag_test.py --output-dir /tmp/tag-views
Requires mujoco, numpy and OpenCV with aruco (opencv-contrib-python-headless).
With AVIATOR_SIMULATION_TEST_EGL=ON and those Python dependencies available at
CMake configuration, this is also registered as simulation_apriltag.

Regenerate the texture (Poppler and Pillow required):
  pdfimages -png docs/Apriltag80id0.pdf /tmp/aviator-tag
  python3 -c 'from PIL import Image; Image.open("/tmp/aviator-tag-002.png").convert("RGB").resize((640,640),Image.NEAREST).save("models/textures/apriltag_36h11_id0.png")'
