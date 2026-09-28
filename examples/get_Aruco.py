# import numpy as np
# import cv2
# from cv2 import aruco
# import pyrealsense2 as rs
# import numpy as np
# # from Talon.Translate import rpy2T,T2rpy,rot2T
# print(cv2.__version__)

# # *********************RealSense Camera*************************
# H = 1280
# W = 720
# Hz = 30
# cam_matrix = np.array([[ 645.3196 , 0, 657.605],
#                 [0, 644.29517, 324.067],
#                 [0, 0, 1]], dtype=np.float32)
# dist_coefficients = np.zeros((4, 1), dtype=np.float32)
# pipeline = rs.pipeline()
# config = rs.config()
# config.enable_stream(rs.stream.color, H, W, rs.format.bgr8, Hz)
# profile = pipeline.start(config)

# # *****************create ChArUco***************************
# squareLength = 0.02    # 实际尺寸 mm
# markerLength = 0.015
# dictionary = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
# board = aruco.CharucoBoard((9, 9), squareLength, markerLength, dictionary)
# # path = './CharucoCalibration/charuco.png'
# # imboard = board.generateImage((1000, 1000))
# # cv2.imwrite(path, imboard)
# # exit()

# detector_params = aruco.CharucoParameters()
# charuco_detector = cv2.aruco.CharucoDetector(board)
# get_intri = True
# while True:
#     # 等待新的帧
#     frames = pipeline.wait_for_frames()
#     color_frame = frames.get_color_frame()
#     color_image = np.asanyarray(color_frame.get_data())
#     if get_intri:
#             intrinsics = color_frame.profile.as_video_stream_profile().intrinsics
#             cam_matrix = np.array([[intrinsics.fx, 0, intrinsics.ppx],
#                             [0, intrinsics.fy, intrinsics.ppy],
#                             [0, 0, 1]], dtype=np.float32)
#             dist_coefficients = np.array(intrinsics.coeffs)
#             get_intri =False
#             print(cam_matrix,dist_coefficients)

#     # *****************ChArUco Detect***************************
#     image_copy = np.copy(color_image)
#     charuco_corners, charuco_ids, marker_corners, marker_ids = charuco_detector.detectBoard(color_image)
#     if not (marker_ids is None) and len(marker_ids) > 0:
#         cv2.aruco.drawDetectedMarkers(image_copy, marker_corners)
#     if not (charuco_ids is None) and len(charuco_ids) > 0:
#         cv2.aruco.drawDetectedCornersCharuco(image_copy, charuco_corners, charuco_ids)
    
#         if len(cam_matrix) > 0 and len(charuco_ids) >= 6:
#             try:
#                 obj_points, img_points = board.matchImagePoints(charuco_corners, charuco_ids)
#                 # flag, rvec, tvec, inliers = cv2.solvePnPRansac(obj_points, img_points, cam_matrix, dist_coefficients)
#                 flag, rvec, tvec = cv2.solvePnP(obj_points, img_points, cam_matrix, dist_coefficients)

#                 if flag:
#                     cv2.drawFrameAxes(image_copy, cam_matrix, dist_coefficients, rvec, tvec, 0.1)
#                     R_ = rvec[:,0]
#                     t_ = tvec[:,0]
#                     # RM_ = rot2T(np.concatenate((t_, R_)))
#                     print('相机坐标系下,Charuco位置:')
#                     # print(inliers)
#                     print(f'X:{t_[0]*1000:.2f}, Y:{t_[1]*1000:.2f}, Z:{t_[2]*1000:.2f}')
#                     print(f'Rx:{np.rad2deg(R_[0]):.2f}, Ry:{np.rad2deg(R_[1]):.2f}, Rz:{np.rad2deg(R_[2]):.2f}')
#                     print('==========================================================')

#             except cv2.error as error_inst:
#                 print("SolvePnP recognize calibration pattern as non-planar pattern. To process this need to use "
#                         "minimum 6 points. The planar pattern may be mistaken for non-planar if the pattern is "
#                         "deformed or incorrect camera parameters are used.")
#                 print(error_inst.err)
#     cv2.imshow("out", image_copy)
#     if cv2.waitKey(1) & 0xFF == ord('q'):
#             break
    
# pipeline.stop()
# cv2.destroyAllWindows()

#!/usr/bin/env python3

import numpy as np
import cv2
from cv2 import aruco
import pyrealsense2 as rs


# =========================
# Camera
# =========================

WIDTH, HEIGHT, FPS = 1280, 720, 30

pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, WIDTH, HEIGHT, rs.format.bgr8, FPS)
profile = pipeline.start(config)

intrinsics = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()

camera_matrix = np.array([
    [intrinsics.fx, 0, intrinsics.ppx],
    [0, intrinsics.fy, intrinsics.ppy],
    [0, 0, 1]
], dtype=np.float32)

dist_coeffs = np.asarray(intrinsics.coeffs, dtype=np.float32).reshape(-1, 1)


# =========================
# ChArUco
# =========================

SQUARE_LENGTH = 0.016
MARKER_LENGTH = 0.015

dictionary = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
board = aruco.CharucoBoard((5, 5), SQUARE_LENGTH, MARKER_LENGTH, dictionary)
detector = aruco.CharucoDetector(board)


# =========================
# Rodrigues -> Quaternion
# qx, qy, qz, qw
# =========================

def rvec_to_quaternion(rvec):
    R, _ = cv2.Rodrigues(rvec)
    qw = np.sqrt(max(0, 1 + np.trace(R))) / 2
    qx = np.sign(R[2, 1] - R[1, 2]) * np.sqrt(max(0, 1 + R[0, 0] - R[1, 1] - R[2, 2])) / 2
    qy = np.sign(R[0, 2] - R[2, 0]) * np.sqrt(max(0, 1 - R[0, 0] + R[1, 1] - R[2, 2])) / 2
    qz = np.sign(R[1, 0] - R[0, 1]) * np.sqrt(max(0, 1 - R[0, 0] - R[1, 1] + R[2, 2])) / 2
    q = np.array([qx, qy, qz, qw])
    return q / np.linalg.norm(q)


# =========================
# Main Loop
# =========================

print(f"RealSense D436: {WIDTH}x{HEIGHT}@{FPS}")
print("Camera Matrix:\n", camera_matrix)
print("Distortion:", dist_coeffs.reshape(-1))

while True:

    frames = pipeline.wait_for_frames()
    color_frame = frames.get_color_frame()
    if not color_frame:
        continue

    image = np.asanyarray(color_frame.get_data())
    display = image.copy()

    charuco_corners, charuco_ids, marker_corners, marker_ids = detector.detectBoard(image)

    # ArUco markers
    if marker_ids is not None:
        cv2.aruco.drawDetectedMarkers(display, marker_corners, marker_ids)

    # ChArUco corners
    if charuco_ids is not None:
        corners = np.asarray(charuco_corners).reshape(-1, 2)
        ids = np.asarray(charuco_ids).reshape(-1)

        for point, idx in zip(corners, ids):
            x, y = int(point[0]), int(point[1])
            cv2.circle(display, (x, y), 4, (0, 255, 0), -1)
            cv2.putText(display, str(int(idx)), (x + 5, y - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1, cv2.LINE_AA)

        # PnP
        if len(charuco_ids) >= 6:

            obj_points, img_points = board.matchImagePoints(charuco_corners, charuco_ids)
            success, rvec, tvec = cv2.solvePnP(obj_points, img_points, camera_matrix, dist_coeffs)

            if success:

                cv2.drawFrameAxes(display, camera_matrix, dist_coeffs, rvec, tvec, 0.1)

                x, y, z = tvec.flatten() * 1000.0
                qx, qy, qz, qw = rvec_to_quaternion(rvec)

                cv2.putText(display, f"X: {x:.2f} mm", (20, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 0), 2)
                cv2.putText(display, f"Y: {y:.2f} mm", (20, 65), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 0), 2)
                cv2.putText(display, f"Z: {z:.2f} mm", (20, 95), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 0), 2)

                cv2.putText(display, f"qx: {qx:.6f}", (20, 135), cv2.FONT_HERSHEY_SIMPLEX, 0.60, (255, 255, 0), 2)
                cv2.putText(display, f"qy: {qy:.6f}", (20, 165), cv2.FONT_HERSHEY_SIMPLEX, 0.60, (255, 255, 0), 2)
                cv2.putText(display, f"qz: {qz:.6f}", (20, 195), cv2.FONT_HERSHEY_SIMPLEX, 0.60, (255, 255, 0), 2)
                cv2.putText(display, f"qw: {qw:.6f}", (20, 225), cv2.FONT_HERSHEY_SIMPLEX, 0.60, (255, 255, 0), 2)

                cv2.putText(display, f"ChArUco: {len(charuco_ids)} points", (20, HEIGHT - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.60, (255, 255, 255), 2)

    cv2.imshow("D436 ChArUco Pose", display)

    key = cv2.waitKey(1) & 0xFF
    if key == ord("q") or key == 27:
        break


# =========================
# Cleanup
# =========================

pipeline.stop()
cv2.destroyAllWindows()
