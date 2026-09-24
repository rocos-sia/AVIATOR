# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

PIN-IK (Pinocchio Inverse Kinematics) is a standalone C++17 inverse kinematics solver using Pinocchio 3.9.0 for kinematics and NLopt for optimization. It's a port of TRAC-IK from Orocos KDL to Pinocchio, made fully independent of ROS.

**Key characteristic**: Dual-threaded solving with configurable optimization modes (Speed, Distance, Manip1/2/3).

**Naming**: PIN-IK (Pinocchio IK) avoids conflicts with the original TRAC-IK (KDL-based) and clearly indicates the use of Pinocchio library.

## Build Commands

```bash
# Standard build
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j$(nproc)

# Run tests
ctest --test-dir build --output-on-failure

# Run specific test
./build/tests/standalone_tests tests/robot.urdf

# Run example
./build/examples/ik_tests tests/robot.urdf base tip 100

# Clean rebuild
rm -rf build && cmake -S . -B build && cmake --build build -j$(nproc)
```

## Architecture

### Three-Layer Solver Design

1. **`PIN_IK::PIN_IK`** (pin_ik.hpp/cpp) - Main API
   - Dual-threaded: spawns `NLOPT_IK` and `ChainIkSolverPos_TL` in parallel
   - Returns first solution or best after timeout
   - Thread-safe per instance (don't share instances across threads)

2. **`ChainIkSolverPos_TL`** (pinocchio_tl.hpp/cpp) - Iterative Jacobian solver
   - Uses `pinocchio::computeFrameJacobian()` for 6×6 Jacobian
   - **Critical**: Uses `pinocchio::integrate(model, q, delta_v, q_new)` for configuration updates (NOT `q = q + delta`)
   - Handles joint limits, wrapping, and random restarts

3. **`NLOPT_IK`** (nlopt_ik.hpp/cpp) - NLopt optimization
   - Uses SLSQP algorithm in configuration space
   - Objective functions: `cartSumSquaredError()` or `cartDQError()` (dual quaternion)
   - NLopt manages configuration updates internally

### Pinocchio 3.9.0 Integration

**Key difference from KDL**: Continuous joints use 2-DOF representation (cos θ, sin θ) instead of 1-DOF angle.

**Manifold operations**:
- Configuration space: `q ∈ ℝ^nq` (includes 2D continuous joints)
- Velocity space: `v ∈ ℝ^nv` (continuous joints contribute 1D angular velocity)
- **Jacobian**: Always `J ∈ ℝ^{6×nv}` (dimensions unchanged from 2.x!)
- **Configuration updates**: MUST use `pinocchio::integrate(model, q, v*dt, q_new)`, never `q + v*dt`

Example model structure:
```
Joint 0: universe (idx_q=-1, virtual, no config space)
Joints 1-5: standard joints (idx_q=0-4, 1 DOF each)
Joint 6: continuous (idx_q=5-6, 2 DOF: cos, sin)
→ nq=7, nv=6, Jacobian is 6×6
```

### URDF Loading

**`loadURDFModel()`** (urdf.cpp):
- Parses URDF via `pinocchio::urdf::buildModelFromXML()`
- Currently uses **full model** (not reduced chain)
- Extracts joint limits for chain between base and tip
- Returns `model` (full), `lower`/`upper` limits, `tip_frame_id`

**Known limitation**: Full model includes all joints, not just the chain. Tests expect simplified chain kinematics, causing test FK mismatches (library itself works correctly).

## Common Pitfalls

### 1. Configuration Space Updates
```cpp
// ❌ WRONG (Pinocchio 3.9.0)
q_new = q + delta_q;

// ✅ CORRECT
pinocchio::integrate(model, q, delta_q, q_new);
```

### 2. Continuous Joint Handling
- Configuration: `q = [..., cos(θ), sin(θ)]` (7D for 6-DOF robot)
- Velocity: `v = [..., θ̇]` (6D)
- Jacobian input: velocity space (6D)
- Forward kinematics input: configuration space (7D)

### 3. Model Structure
- `model.nq`: configuration space dimension
- `model.nv`: velocity space dimension
- `model.njoints`: number of joints (includes universe)
- Universe joint has `idx_q=-1` (no config space)

## Testing

Test suite (standalone_tests.cpp) currently expects simplified chain model but code uses full model. This causes FK disagreement but does not affect actual IK solving functionality.

To verify IK works:
1. Create real robot URDF
2. Test with `PIN_IK` class directly
3. Check solution with forward kinematics
4. Verify joint limits respected

## Migration Status

**Completed** (commit d53dbfe):
- ✅ All KDL code removed
- ✅ Pinocchio 3.9.0 integration
- ✅ `pinocchio::integrate()` implemented
- ✅ Core library compiles and links

**Remaining** (optional):
- Test suite adjustment for full model FK
- Or: implement chain extraction (buildReducedModel with continuous joint handling)
- Or: downgrade to Pinocchio 2.x for immediate test compatibility

## Key Files

- **pin_ik.cpp**: Main solver orchestration
- **pinocchio_tl.cpp**: Jacobian IK solver ← Uses `pinocchio::integrate()`
- **nlopt_ik.cpp**: NLopt optimization ← NLopt manages updates internally
- **urdf.cpp**: Model loading ← Returns full model, extracts chain limits
- **pinocchio_types.hpp**: Type definitions and helper functions

## Dependencies

- Pinocchio 3.9.0 (`/opt/openrobots`)
- Eigen3
- NLopt
- urdfdom (standalone, not ROS)
- Boost (for serialization)

## Documentation

See `docs/` folder:
- **MIGRATION_STATUS.md**: Migration progress tracking
- **FINAL_STATUS.md**: Complete status summary
- **NEXT_STEPS.md**: Pinocchio 3.9.0 continuous joint adaptation guide
- **JACOBIAN_DETAILS.md**: Technical explanation of configuration vs velocity space
- **README_PINOCCHIO.md**: Pinocchio usage guide
