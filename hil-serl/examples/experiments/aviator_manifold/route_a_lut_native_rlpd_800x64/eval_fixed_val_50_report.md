# Fixed 50-trajectory validation: fresh 800×64 RLPD

Validation source: `data/aviator/trajectory_source/trajs/val/traj_0000.npz` through `traj_0049.npz`. All checkpoints use deterministic SAC actions and the same unshielded `AviatorManifoldEnv`.

| Checkpoint step | Completed / 50 | Clearance failures | Speed failures | Grid exits | Mean return | Worst clearance (mm) |
|---:|---:|---:|---:|---:|---:|---:|
| 20000 | 8 | 33 | 9 | 0 | 233.60 | 4.00 |
| 40000 | 30 | 7 | 13 | 0 | 540.57 | 4.48 |
| 60000 | 49 | 0 | 1 | 0 | 822.20 | 6.11 |
| 80000 | 47 | 2 | 1 | 0 | 790.04 | 4.74 |
| 100000 | 41 | 6 | 3 | 0 | 767.01 | 4.32 |
| 120000 | 42 | 7 | 1 | 0 | 773.92 | 4.42 |
| 140000 | 25 | 23 | 2 | 0 | 477.71 | 4.43 |
| 160000 | 42 | 7 | 1 | 0 | 778.78 | 4.69 |
| 180000 | 48 | 1 | 1 | 0 | 797.51 | 4.81 |
| 200000 | 47 | 2 | 1 | 0 | 805.16 | 4.94 |
| 220000 | 29 | 16 | 1 | 4 | 556.11 | 4.19 |
| 240000 | 29 | 18 | 2 | 1 | 593.04 | 4.52 |
| 260000 | 39 | 9 | 1 | 1 | 726.97 | 3.02 |
| 280000 | 21 | 22 | 5 | 2 | 464.99 | 3.55 |
| 300000 | 5 | 24 | 20 | 1 | 165.73 | 2.14 |
| 320000 | 0 | 49 | 1 | 0 | -39.69 | 0.37 |
| 322445 | 0 | 49 | 1 | 0 | -55.49 | 0.32 |

Best by completion, then mean return: checkpoint_60000 (49/50; mean return 822.20).
Final checkpoint_322445: 0/50; 49 clearance failures and 1 speed failure.
The standard repository CLI independently reproduced the 60k and final results; logs: `eval_best_cli.log`, `eval_final_cli.log`.
Raw per-trajectory outcomes: `eval_fixed_val_50.json`; compact table: `eval_fixed_val_50.csv`.
