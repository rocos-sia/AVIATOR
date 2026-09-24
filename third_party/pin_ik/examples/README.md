# PIN-IK Standalone Examples

Build from the repository root, then run:

```sh
build/examples/ik_tests tests/robot.urdf base tip 100
```

Arguments: URDF file, base link, tip link, optional number of random samples.

Every successful solution is checked against forward kinematics and joint limits.

The example demonstrates PIN-IK's dual-threaded solving with random target poses.
