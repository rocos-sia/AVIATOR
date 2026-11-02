#!/usr/bin/env python3
"""Derive short-wrist hand meshes without changing the original CAD exports.

The original base STL includes a separate, closed cylindrical wrist solid from
Z=0 to 38.5 mm. The palm starts at Z=30.5 mm. Remove that complete solid and
translate the remaining solids by -30.5 mm; no mesh scaling or open cuts.
"""
import argparse
from pathlib import Path
import struct

ROOT = Path(__file__).resolve().parents[1]
PALM_OFFSET = 0.0305
FACE = struct.Struct('<12fH')


def shorten(source):
    raw = source.read_bytes()
    count = struct.unpack_from('<I', raw, 80)[0]
    if len(raw) != 84 + count * FACE.size:
        raise ValueError(f'Invalid binary STL: {source}')
    faces = list(FACE.iter_unpack(raw[84:]))
    parents = []
    vertices = {}
    face_vertices = []

    def root(index):
        while parents[index] != index:
            parents[index] = parents[parents[index]]
            index = parents[index]
        return index

    for face in faces:
        indices = []
        for start in (3, 6, 9):
            vertex = face[start:start + 3]
            if vertex not in vertices:
                vertices[vertex] = len(parents)
                parents.append(len(parents))
            indices.append(vertices[vertex])
        for index in indices[1:]:
            parents[root(index)] = root(indices[0])
        face_vertices.append(indices[0])
    components = {}
    for vertex, index in vertices.items():
        components.setdefault(root(index), []).append(vertex)
    wrists = [index for index, points in components.items()
              if min(p[2] for p in points) < PALM_OFFSET - 1e-6]
    if len(wrists) != 1:
        raise ValueError('Expected exactly one separate original wrist solid')
    wrist = wrists[0]
    points = components[wrist]
    if (abs(min(p[2] for p in points)) > 1e-6
            or abs(max(p[2] for p in points) - .0385) > 1e-6
            or max(p[0] ** 2 + p[1] ** 2 for p in points) > .0251 ** 2):
        raise ValueError('Original wrist dimensions differ from the expected CAD export')
    output = []
    for face, vertex in zip(faces, face_vertices):
        if root(vertex) == wrist:
            continue
        shifted = list(face)
        for index in (5, 8, 11):
            shifted[index] -= PALM_OFFSET
        output.append(FACE.pack(*shifted))
    header = b'AVIATOR short wrist: original wrist solid removed; palm Z -= 30.5 mm'
    return header.ljust(80, b'\0') + struct.pack('<I', len(output)) + b''.join(output)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--check', action='store_true', help='Verify generated assets without writing')
    args = parser.parse_args()
    for side, prefix in (('L', 'l'), ('R', 'r')):
        source = ROOT / f'models/meshes/RH56E2_{side}/{prefix}_base_link.STL'
        destination = source.parent / 'short_wrist' / source.name
        result = shorten(source)
        if args.check:
            if not destination.is_file() or destination.read_bytes() != result:
                raise SystemExit(f'Short-wrist asset is missing or stale: {destination}')
        else:
            destination.parent.mkdir(exist_ok=True)
            destination.write_bytes(result)
        print(f'{destination.relative_to(ROOT)}: {struct.unpack_from("<I", result, 80)[0]} faces')


if __name__ == '__main__':
    main()
