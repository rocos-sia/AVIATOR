"""Render and decode the yoke's PDF-derived tag throughout its joint limits.

Requires MuJoCo, NumPy, OpenCV with aruco, and working EGL rendering.
Run: MUJOCO_GL=egl python3 tests/mjcf_apriltag_test.py [--output-dir /tmp/tag-views]
"""
import argparse
import os
from pathlib import Path

os.environ.setdefault("MUJOCO_GL", "egl")

import cv2
import mujoco as mj
import numpy as np

ROOT = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path)
    args = parser.parse_args()
    if args.output_dir:
        args.output_dir.mkdir(parents=True, exist_ok=True)
    model = mj.MjModel.from_xml_path(str(ROOT / "models/mjcf/aviator.xml"))
    data = mj.MjData(model)
    parameters = cv2.aruco.DetectorParameters()
    parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_APRILTAG
    detector = cv2.aruco.ArucoDetector(
        cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11), parameters)
    tag = model.geom("yoke_apriltag").id
    wheel = model.body("steering_wheel").id
    camera = model.camera("cockpit_apriltag").id
    roll = model.joint("roll_input_joint").id
    pitch = model.joint("pitch_input_joint").id
    assert model.geom_bodyid[tag] == wheel
    assert model.geom_contype[tag] == model.geom_conaffinity[tag] == 0
    width, height = 640, 480
    focal = .5 * height / np.tan(np.deg2rad(model.cam_fovy[camera]) / 2)
    camera_pose = None
    # Black border is 80 mm wide; the full texture includes a 10 mm white margin.
    border = np.array([[-.04, -.04, .0005], [.04, -.04, .0005],
                       [.04, .04, .0005], [-.04, .04, .0005]])
    board = border.copy()
    board[:, :2] *= 1.25  # Also check the entire white margin for occlusion.
    with mj.Renderer(model, height=height, width=width) as renderer:
        for i, angle in enumerate(np.linspace(*model.jnt_range[roll], 11)):
            for j, displacement in enumerate([model.jnt_range[pitch, 0], -.085,
                                              model.jnt_range[pitch, 1]]):
                mj.mj_resetDataKeyframe(model, data, model.key("aviator_home").id)
                data.qpos[model.jnt_qposadr[roll]] = angle
                data.qpos[model.jnt_qposadr[pitch]] = displacement
                mj.mj_forward(model, data)
                pose = np.r_[data.cam_xpos[camera], data.cam_xmat[camera]]
                if camera_pose is None:
                    camera_pose = pose.copy()
                np.testing.assert_allclose(pose, camera_pose, atol=1e-12)
                renderer.update_scene(data, camera="cockpit_apriltag")
                rgb = renderer.render()
                corners, ids, _ = detector.detectMarkers(rgb)
                if args.output_dir:
                    cv2.imwrite(str(args.output_dir / f"tag_roll{i}_pitch{j}.png"),
                                cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))
                assert ids is not None and 0 in ids, (angle, displacement, ids)
                observed = corners[list(ids.ravel()).index(0)].reshape(4, 2)
                world = border @ data.geom_xmat[tag].reshape(3, 3).T + data.geom_xpos[tag]
                view = (world - data.cam_xpos[camera]) @ data.cam_xmat[camera].reshape(3, 3)
                projected = np.column_stack((width/2 - focal * view[:, 0]/view[:, 2],
                                              height/2 + focal * view[:, 1]/view[:, 2]))
                # Catch incorrect texture scale/UV mapping as well as an undecodable mirror.
                errors = np.linalg.norm(projected[:, None] - observed[None, :], axis=2)
                assert np.max(np.min(errors, axis=1)) < 3, errors
                board_world = board @ data.geom_xmat[tag].reshape(3, 3).T + data.geom_xpos[tag]
                board_view = (board_world - data.cam_xpos[camera]) @ data.cam_xmat[camera].reshape(3, 3)
                polygon = np.column_stack((width/2 - focal * board_view[:, 0]/board_view[:, 2],
                                           height/2 + focal * board_view[:, 1]/board_view[:, 2]))
                assert np.all((polygon > [0, 0]) & (polygon < [width, height])), polygon
                mask = np.zeros((height, width), np.uint8)
                cv2.fillConvexPoly(mask, np.rint(polygon).astype(np.int32), 1)
                # Ignore two raster-edge pixels: rounding/MSAA do not indicate occlusion.
                mask = cv2.erode(mask, np.ones((5, 5), np.uint8)).astype(bool)
                renderer.enable_segmentation_rendering()
                segmentation = renderer.render()
                renderer.disable_segmentation_rendering()
                visible = ((segmentation[:, :, 0] == tag) &
                           (segmentation[:, :, 1] == int(mj.mjtObj.mjOBJ_GEOM)))
                occluded = 1 - np.mean(visible[mask])
                assert occluded <= .001, (angle, displacement, occluded)
                print(f"PASS roll={angle:.5f}, pitch={displacement:.3f}: ID 0, 80 mm, board visible")
    print(f"PASS 33 rendered poses on MuJoCo {mj.__version__}; camera fixed, tag follows wheel")


if __name__ == "__main__":
    main()
