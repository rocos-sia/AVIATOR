"""Export the selected Flax actor to an ONNX-conversion parameter blob.

Run with the serl_clean Python environment, then run actor_to_onnx.py.
Only the deterministic mean head is exported; the critic, optimizer and
variance head stay in Flax. The C++ runtime loads the resulting ONNX file.
"""
import argparse
import struct
from pathlib import Path

import numpy as np
from flax.training import checkpoints

ROOT = Path(__file__).resolve().parents[2]
DEFAULT_CHECKPOINT = (ROOT / "hil-serl/examples/experiments/aviator_manifold"
                      / "route_a_lut_native_rlpd_800x64/checkpoint_60000")


def export(checkpoint: Path, output: Path):
    state = checkpoints.restore_checkpoint(str(checkpoint), target=None)
    if not isinstance(state, dict) or "params" not in state:
        raise ValueError(f"Could not restore Flax state from {checkpoint}")
    actor = state["params"]["modules_actor"]
    net = actor["network"]
    layers = []
    for i, (n_in, n_out) in enumerate(((40, 256), (256, 256), (256, 256))):
        dense = net[f"Dense_{i}"]
        norm = net[f"LayerNorm_{i}"]
        arrays = (dense["kernel"], dense["bias"], norm["scale"], norm["bias"])
        expected = ((n_in, n_out), (n_out,), (n_out,), (n_out,))
        for arr, shape in zip(arrays, expected):
            if np.shape(arr) != shape:
                raise ValueError(f"Layer {i}: expected {shape}, got {np.shape(arr)}")
        layers.extend(arrays)
    mean = actor["Dense_0"]
    if np.shape(mean["kernel"]) != (256, 2) or np.shape(mean["bias"]) != (2,):
        raise ValueError("Unexpected actor mean-head shape")
    layers.extend((mean["kernel"], mean["bias"]))
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open("wb") as f:
        f.write(struct.pack("<8s4I", b"T6ACTOR\0", 1, 40, 256, 2))
        for array in layers:
            f.write(np.asarray(array, dtype="<f4").tobytes(order="C"))
    return output


if __name__ == "__main__":
    p = argparse.ArgumentParser()
    p.add_argument("--checkpoint", type=Path, default=DEFAULT_CHECKPOINT)
    p.add_argument("--output", type=Path, default=Path(__file__).with_name("actor_60k.bin"))
    args = p.parse_args()
    print(export(args.checkpoint, args.output))
