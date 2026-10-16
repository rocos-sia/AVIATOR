"""Derive the 50 mm narrower main yoke from the original CAD STL assets.

Run from any directory with Python, NumPy and SciPy. Original assets are kept
for reproducibility and for the historical examples that still reference them.
The secondary cockpit yoke is unchanged.
"""
from pathlib import Path
import struct

import numpy as np
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import connected_components

ROOT = Path(__file__).resolve().parents[1] / "models/meshes/Cessna"
RECORD = np.dtype([("normal", "<f4", 3), ("vertices", "<f4", (3, 3)),
                   ("attribute", "<u2")])
INSET = 0.025


def shorten_bridge(vertices):
    # Keep the shaft attachment (|X| <= 35 mm) fixed. Shorten only the
    # connecting spans; beyond 90 mm translate the complete grip assembly.
    x = vertices[..., 0].astype(float)
    vertices[..., 0] = x - np.sign(x) * INSET * np.clip((abs(x) - .035) / .055, 0, 1)


def narrow(source, destination, assembly=False):
    raw = source.read_bytes()
    count = struct.unpack_from("<I", raw, 80)[0]
    if len(raw) != 84 + 50 * count:
        raise ValueError(f"Expected a binary STL: {source}")
    records = np.frombuffer(raw, dtype=RECORD, offset=84).copy()
    vertices = records["vertices"]
    if assembly:
        points, indices = np.unique(vertices.reshape(-1, 3), axis=0, return_inverse=True)
        faces = indices.reshape(-1, 3)
        graph = coo_matrix((np.ones(faces.size),
                            (faces.ravel(), np.roll(faces, 1, axis=1).ravel())),
                           shape=(len(points), len(points)))
        count, labels = connected_components(graph, directed=False)
        if count != 24:
            raise ValueError("Source CAD topology changed; review component classification")
        for component in range(count):
            mask = labels[faces[:, 0]] == component
            part = vertices[mask].copy()
            lo, hi = part[..., 0].min(), part[..., 0].max()
            if hi < -.03:
                part[..., 0] += INSET
            elif lo > .03:
                part[..., 0] -= INSET
            elif hi - lo > .2:
                shorten_bridge(part)
            # Central shaft, mounting hardware and AprilTag plate stay fixed.
            vertices[mask] = part
    else:
        shorten_bridge(vertices)
    changed = np.any(vertices != np.frombuffer(raw, dtype=RECORD, offset=84)["vertices"],
                     axis=(1, 2))
    triangles = vertices[changed].astype(float)
    normals = np.cross(triangles[:, 1] - triangles[:, 0], triangles[:, 2] - triangles[:, 0])
    lengths = np.linalg.norm(normals, axis=1)
    normals /= np.where(lengths > 0, lengths, 1)[:, None]
    records["normal"][changed] = normals
    destination.parent.mkdir(parents=True, exist_ok=True)
    destination.write_bytes(raw[:84] + records.tobytes())
    print(destination.relative_to(ROOT), f"({int(changed.sum())} moved triangles)")


if __name__ == "__main__":
    narrow(ROOT / "steering_wheel_qr.STL", ROOT / "narrow/steering_wheel_qr.STL", True)
    for index in (29, 43):
        name = f"steering_wheel_collision_{index}.stl"
        narrow(ROOT / name, ROOT / "narrow" / name)
