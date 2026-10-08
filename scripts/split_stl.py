"""Split a binary STL into MuJoCo-compatible chunks without changing facets."""
import argparse
import struct
from pathlib import Path


MAX_FACES = 200000


def split_stl(source, output_dir):
    data = source.read_bytes()
    if len(data) < 84:
        raise ValueError(f"Invalid binary STL header: {source}")
    count = struct.unpack_from("<I", data, 80)[0]
    if count == 0 or len(data) != 84 + count * 50:
        raise ValueError(f"Invalid binary STL face count or length: {source}")
    outputs = [
        output_dir / f"{source.stem}_{index}{source.suffix}"
        for index in range((count + MAX_FACES - 1) // MAX_FACES)
    ]
    for path in outputs:
        if path.exists():
            raise FileExistsError(f"Refusing to overwrite {path}")
    output_dir.mkdir(parents=True, exist_ok=True)
    for index, path in enumerate(outputs):
        start = index * MAX_FACES
        size = min(MAX_FACES, count - start)
        chunk = data[:80] + struct.pack("<I", size) + data[84 + start * 50:84 + (start + size) * 50]
        with path.open("xb") as output:
            output.write(chunk)
        print(f"{path}: {size} faces, {len(chunk)} bytes")
    return outputs


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source", type=Path)
    parser.add_argument("output_dir", type=Path)
    args = parser.parse_args()
    split_stl(args.source, args.output_dir)


if __name__ == "__main__":
    main()
