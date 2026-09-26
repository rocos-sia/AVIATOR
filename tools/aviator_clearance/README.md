# aviator_clearance — `build_manifold_phi` (PIN-IK / Pinocchio port)

Independent, minimal-footprint port of the `build_manifold_phi` manifold
builder (`clearance_trajectory.cpp`) from the `grasp-conditioned` branch.

## What changed vs. the original

| Original (`grasp-conditioned`) | Port (this dir) |
|---|---|
| `kdl_parser::treeFromFile` + `KDL::Chain` | `PIN_IK::loadURDFModel` (Pinocchio chain) |
| `TRAC_IK::TRAC_IK` / `TRAC_IK::Speed` | `PIN_IK::PIN_IK` / `PIN_IK::Speed` |
| `KDL::Vector` / `KDL::Rotation` / `KDL::Frame` / `KDL::JntArray` | `kdl_compat.hpp` shim (Eigen / `pinocchio::SE3`) |

The IK backend is the only semantic change. The KDL-shaped math surface the
tool actually uses (`Frame`, `Rotation` with `Rot`/`RotZ`/`Quaternion`/`GetRot`,
`Vector`, `JntArray`) is re-provided by [`kdl_compat.hpp`](kdl_compat.hpp) as
thin wrappers over Eigen and `pinocchio::SE3` — so `clearance_trajectory.cpp`
kept its math verbatim; only the includes, the URDF-chain setup, and the
`TRAC_IK` → `PIN_IK` rename were edited. No KDL / kdl_parser / TRAC-IK symbols
remain.

## Build (standalone — not part of dev's main build tree)

```sh
cmake -S tools/aviator_clearance -B build/aviator_clearance \
      -DAVIATOR_DEPENDENCY_JOBS=$(nproc)
cmake --build build/aviator_clearance -j$(nproc)
```

This builds the vendored Coal / Pinocchio / PIN-IK / MuJoCo libraries into
`build/aviator_clearance/third_party/install` and produces
`build/aviator_clearance/bin/build_manifold_phi`. Dev's own build tree is not
touched.

## Run

Identical to the original tool — it reads a config YAML (`urdf`, `model`,
`grasp`, `posture` keys). Point it at the same config/model/URDF as the
`grasp-conditioned` branch:

```sh
build/aviator_clearance/bin/build_manifold_phi <config.yaml>
```

## Notes

- MuJoCo joint limits (with the J2 planning margin from `posture.json`) are
  still what feeds PIN-IK's `q_min`/`q_max`, matching the original TRAC-IK path;
  the URDF limits returned by `loadURDFModel` are not used.
- The wall geoms are optional: if the MJCF has no `aviator_wall` geoms (as in
  dev's `examples/AviatorRobot_simple/model/aviator.xml`), `wall_clearance`
  returns 1.0 and the static-envelope/clearance terms degrade gracefully.
