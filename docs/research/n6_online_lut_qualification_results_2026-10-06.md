# N6 Online continuation versus LUT: qualification results

## Scope

N6 compared the frozen lookup-table pipeline with the online local continuation under the same task commands, sampled initial states, MuJoCo checker, `dt=0.005`, 100 steps, and `d_safe=5` mm. The frozen-LUT policy was corrected before the formal run: at every step it scales the nominal phase action along one dimension until the interpolated safe interval is respected. The earlier fixed-phase runs in `n6_smoke2` and `n6_online_lut_qualification_v1` are diagnostic only and are not used as evidence.

The online method uses the frozen three-variable QP, governor, midpoint integration, and independent nonlinear endpoint/swept checker. The LUT method uses no task governor, Jacobian, or QP; it is rejected when the independent checker rejects its next state.

## Formal outputs

- Corrected frozen-table comparison: `outputs/differential_lift_feasibility_20261006/n6_online_lut_qualification_v2/`
- Rebuilt nominal table comparison: `outputs/differential_lift_feasibility_20261006/n6_rebuilt_compare_v2_nominal/`
- Rebuilt tight table comparison: `outputs/differential_lift_feasibility_20261006/n6_rebuilt_compare_v2_tight/`
- Rebuilt shifted table comparison: `outputs/differential_lift_feasibility_20261006/n6_rebuilt_compare_v2_shifted/`
- Combined immutable summary: `outputs/differential_lift_feasibility_20261006/n6_online_lut_qualification_v2/n6_qualification_summary_final_v2.json`
- Cell comparison table: `outputs/differential_lift_feasibility_20261006/n6_online_lut_qualification_v2/n6_comparison_table.csv`

Each formal stale-table cell has 10 paired trials (90 pairs total). Each rebuilt-table scene has 30 paired trials (10 per speed).

## Results

### Frozen stale table versus online continuation

| Method | Complete | Median progress | Rejected clearance candidates | Median step latency |
|---|---:|---:|---:|---:|
| Frozen safe-scaled LUT | 52/90 (57.8%) | 1.00 | 38 | 10.46 ms |
| Online lift + QP + checker | 42/90 (46.7%) | 0.95 | 0 | 67.15 ms |

The online method used 717 governed steps and stopped 46 times at endpoint validation plus twice at a post-integrator check. The LUT stopped 38 times at endpoint validation. The LUT's `rejected clearance candidates` count records candidate states rejected before acceptance; it is not a count of accepted physical collisions. Both methods therefore had zero accepted collision states in this finite-step protocol.

The only perturbation where the online method exceeded the frozen LUT in completion was the stale-table tight scene: online completed 10/30 and the LUT completed 8/30. The online method also had zero rejected clearance candidates there, while the LUT had 18. This is conditional evidence that local continuation can remain usable when a table is frozen against a changed obstacle geometry.

The result is not uniform. In nominal and shifted scenes, the frozen LUT completed 22/30 in each scene while online continuation completed 15/30 in each. The online method also costs about 6.4 times the median per-step latency in this implementation.

### Rebuilt tables

Rebuilding the table recovered the following completion rates:

| Geometry | Rebuilt LUT | Online continuation |
|---|---:|---:|
| Nominal | 22/30 (73.3%) | 15/30 (50.0%) |
| Tight | 21/30 (70.0%) | 13/30 (43.3%) |
| Shifted | 22/30 (73.3%) | 15/30 (50.0%) |

The three rebuilt tables each occupied about 186.03 MiB and took 51.75--52.23 s to build in the recorded environment. These rebuilt runs use independently sampled states from the corresponding rebuilt table and are a cost/recovery qualification, not a cross-table paired hypothesis test.

## Interpretation

N6 does **not** establish that online continuation is non-inferior to a rebuilt LUT, faster, or better in completion rate. It does not justify a general claim of adaptation superiority. It does support a narrower engineering statement:

> A local online continuation can avoid rebuilding a task-specific table and can produce auditable nonlinear safety stops. Under the tested tight stale-table perturbation it retained more progress than the frozen table, but this benefit was conditional and came with substantially higher online latency and lower completion than a rebuilt table.

The paper should therefore retain LUT comparison as a negative/conditional qualification rather than a headline superiority claim. Any claim about reconstruction cost must state the measured table size and build time and must not imply that the online controller has lower total computational cost in the current implementation.

## Limitations

1. The LUT comparator is a safe-scaled phase replay, not the full learned T6 policy or a dynamically optimized table controller.
2. Rebuilt-table runs were sampled separately from the stale-table run; they qualify recovery and construction cost, but do not provide a paired stale-versus-rebuilt statistical test.
3. The scenes test three fixed obstacle variants and three task speeds; no claim is made for arbitrary geometry or calibration drift.
4. Physical execution remains untested.

