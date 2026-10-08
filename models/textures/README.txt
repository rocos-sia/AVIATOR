apriltag_36h11_id0.png
Source: docs/Apriltag120id0.pdf (tag36h11, ID 0, black border size 120 mm).
Extracted the embedded 10x10 RGB image using Poppler pdfimages -png (image 002),
then enlarged to 640x640 with nearest-neighbor sampling. No marker bits changed.
The outer one-cell white margin is retained: a 150 mm board gives a 120 mm tag,
with 15 mm of white margin on each side. The PDF draws the 10x10 image at
425.1968503937 points (150 mm); the inner 8x8 black-border square is 120 mm.
The 80 mm and 120 mm PDFs contain identical marker pixels, so the PNG is unchanged;
physical size comes from the MJCF geom, not the PNG resolution.
MuJoCo material uses one 2D texture repeat and no specular reflection.

Mount: models/mjcf/aviator.xml, steering_wheel child geom yoke_apriltag.
Local +Z is the shaft axis; local +Y is up at neutral roll. The texture overlay
is centered at (0.0004157, 0.11779, 0.32265) m above steering_wheel_qr.STL's plate.
The plate front is at Z=0.32111523 m; the overlay rear is at Z=0.32215 m.
The 1 mm outward adjustment from the previous 80 mm mounting position prevents
the enlarged white margin from intersecting the surrounding wheel mesh.
Front-center site yoke_apriltag_center = (0.0004157, 0.11779, 0.32315) m.
No additional cylindrical standoff is used. The 150 mm paper/texture extends
beyond the approximately 120 mm CAD mounting plate; the wheel mesh is not scaled.
The board adds no mass or collision contacts. Both roll_input_joint and
pitch_input_joint move the tag, including its offset from the rotation axis.
Visibility is checked from cockpit_apriltag, not guaranteed from arbitrary views.

Fixed MJCF camera cockpit_apriltag faces the tag from the pilot side. Its virtual
mount is 100 mm higher than the previous model and re-aimed at the new board
to clear the cockpit shell crossbar over the full wheel travel; this is not a
calibration of the physical camera embedded in the URDF visual mesh. The simulation
camera uses it when present and retains the legacy view for other models.
The existing simulation camera.detection messages still use model ground truth;
adding a tag does not switch that protocol to an image-based AprilTag detector.
The actual PDF texture is independently decoded from rendered images in the test:
  MUJOCO_GL=egl python3 tests/mjcf_apriltag_test.py --output-dir /tmp/tag-views
Requires mujoco, numpy and OpenCV with aruco (opencv-contrib-python-headless).
With AVIATOR_SIMULATION_TEST_EGL=ON and those Python dependencies available at
CMake configuration, this is also registered as simulation_apriltag.

Regenerate the texture (Poppler and Pillow required):
  pdfimages -png docs/Apriltag120id0.pdf /tmp/aviator-tag
  python3 -c 'from PIL import Image; Image.open("/tmp/aviator-tag-002.png").convert("RGB").resize((640,640),Image.NEAREST).save("models/textures/apriltag_36h11_id0.png")'
