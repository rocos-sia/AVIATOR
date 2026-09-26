"""Convert export_actor.py's versioned parameter blob to an ONNX actor.

The ONNX graph exactly represents the trained deterministic policy:
Dense -> LayerNorm(eps=1e-6) -> Tanh, repeated three times, then Dense -> Tanh.
"""
import argparse
import struct
from pathlib import Path

import numpy as np
import onnx
from onnx import TensorProto, helper, numpy_helper

HERE = Path(__file__).resolve().parent


def convert(source: Path, output: Path):
    raw = source.read_bytes()
    magic, version, inputs, width, outputs = struct.unpack_from("<8s4I", raw)
    if (magic, version, inputs, width, outputs) != (b"T6ACTOR\0", 1, 40, 256, 2):
        raise ValueError("Unexpected actor parameter format")
    floats = np.frombuffer(raw, dtype="<f4", offset=24)
    cursor = 0
    nodes, initializers = [], []

    def take(name, shape):
        nonlocal cursor
        count = int(np.prod(shape))
        if cursor + count > len(floats):
            raise ValueError("Truncated actor parameters")
        array = np.array(floats[cursor:cursor + count].reshape(shape), dtype=np.float32)
        cursor += count
        initializers.append(numpy_helper.from_array(array, name))
        return name

    x = "state"
    for i, n_in in enumerate((40, 256, 256)):
        weight = take(f"layer{i}_weight", (n_in, 256))
        bias = take(f"layer{i}_bias", (256,))
        scale = take(f"layer{i}_scale", (256,))
        offset = take(f"layer{i}_offset", (256,))
        dense = f"layer{i}_dense"
        shifted = f"layer{i}_shifted"
        normalized = f"layer{i}_normalized"
        activated = f"layer{i}_activated"
        nodes.extend((
            helper.make_node("MatMul", [x, weight], [dense]),
            helper.make_node("Add", [dense, bias], [shifted]),
            helper.make_node("LayerNormalization", [shifted, scale, offset],
                             [normalized], axis=-1, epsilon=1e-6),
            helper.make_node("Tanh", [normalized], [activated]),
        ))
        x = activated
    weight = take("mean_weight", (256, 2))
    bias = take("mean_bias", (2,))
    nodes.extend((
        helper.make_node("MatMul", [x, weight], ["mean_dense"]),
        helper.make_node("Add", ["mean_dense", bias], ["mean_shifted"]),
        helper.make_node("Tanh", ["mean_shifted"], ["action"]),
    ))
    if cursor != len(floats):
        raise ValueError("Unexpected trailing actor parameters")
    graph = helper.make_graph(
        nodes, "aviator_t6_deterministic_actor",
        [helper.make_tensor_value_info("state", TensorProto.FLOAT, [1, 40])],
        [helper.make_tensor_value_info("action", TensorProto.FLOAT, [1, 2])],
        initializer=initializers)
    model = helper.make_model(graph, opset_imports=[helper.make_opsetid("", 17)],
                              producer_name="aviator_t6_actor_to_onnx")
    model.ir_version = 9
    onnx.checker.check_model(model)
    output.parent.mkdir(parents=True, exist_ok=True)
    onnx.save(model, output)
    return output


if __name__ == "__main__":
    p = argparse.ArgumentParser()
    p.add_argument("--source", type=Path, default=HERE / "actor_60k.bin")
    p.add_argument("--output", type=Path, default=HERE / "actor_60k.onnx")
    args = p.parse_args()
    print(convert(args.source, args.output))
