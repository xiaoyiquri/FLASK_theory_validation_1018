# FLASK first-solution / fixed-budget measurement contract

The recorded objective is `J = rho*T + integral(w^T R w dt)`, with flat input
`w`; `T` is trajectory execution duration. The horizontal axis of the cost-time
plot is planning wall time, not execution time. No new search algorithm,
candidate-time multiplication, biased sampling or geometric-length optimizer
is introduced.

## Events and budgets

`planner.solve` starts one clock for search and Algorithm 4. A first event is
recorded only after the complete extracted path passes validation. Each
accepted, cost-decreasing shortcut records its actual observation time and
an immutable copy of the entire current path. A path checked after the total
deadline cannot be published as an in-budget incumbent. Operations themselves
are synchronous: a final operation may overrun the wall budget, but cannot
retroactively contribute to an earlier checkpoint.

All length integration, physical diagnostics, YAML/sample export and native
checker calls occur after that planning clock. Event recording introduces
bookkeeping overhead inside planning; measurements include it. First metrics
are taken from the first snapshot together. Final metrics are taken from the
last accepted snapshot. Each path has its own cost, execution duration, length
and availability time. The raw first path is never replaced by a later path.

At checkpoint `b`, select the last event with `available_seconds <= b`.
Before the first event there is no solution: cost/length are null, not zero.
When RRTConnect and its one shortcut pass stop early, hold the last incumbent
at later checkpoints; do not restart or imply continued optimization. Default
checkpoints: .001/.005/.01/.05/.1/.5/1/5/10 seconds, total budget 10 seconds.
Default seeds are 20 through 39, one run per seed and scene.

Per-run incumbent costs decrease, but the conditional mean over runs that
already have solutions can increase as additional runs become successful.
Plots display individual step functions and the conditional mean together
with success fraction, retaining all failures in its denominator. Process
startup, export and checker latency do not become first-solution time.

## Source files and model distinction

Original environments in `examples/dynobench_data` specify obstacles,
workspace bounds and complete physical start/goal states. Unmodified model
snapshots under `examples/dynobench_original_models` specify visible native
dimensions and arm lengths. Files have SHA256 manifests. Execution embeds
environment, planning-model and original-model snapshots into every saved
trajectory, so later plotting never substitutes an edited current scene.

The user explicitly retained paper dynamics. Differences from original
DynoBench Unicycle and Quad3D models are exported in `model_differences.json`.
The planner's robot sphere and obstacle sphere cover are still explicit local
approximations; drawing a native box/sphere does not certify that geometry.
Body poses are derived from actual physical samples; raw start/goal model
poses always come from the environment file, including when reduced boundary
attitudes differ.

## Theory-to-code additions

| Contract / requested review | Implementation | Verification | Scope and exceptions |
| --- | --- | --- | --- |
| Directed positive-time BVP and range interpolation | `bvp.h`, `interpolateForTree`, `extend` | SPD/Gramian energy tests, complete endpoints, outgoing prefix and incoming suffix re-steering | Range interpolation is a BVP time fraction, not a projection onto Eq.33's cost neighborhood |
| Connect/traceback must make a continuous directed flat path | `extend` tests actual flat-state equality before Reached; `extract` orders start and goal edges | Planner test validates every saved event as a whole path and both endpoints | Full physical continuity additionally depends on model/order and configured contract |
| Algorithm 4 mutable segment indices | `shorten` restarts j at current M after an accepted splice; accepts lower LQMT objective | Every event validated; event count equals accepted shortcuts + first path; J strictly decreases | Attempt/time caps are explicit local parameters; J decrease does not imply shorter geometric length |
| Scalar/SIMD constraint agreement | Shared physical parameters, aligned ABI, derivative/physical/collision batch masks | Random normalized/non-normalized curves, inactive tails, zero speed/force, angular-rate boundary and full endpoint grids | Double4x2; finite checking does not prove continuous validity |
| Failure accounting | Interval reason propagated; Numeric, SpeedZero/Max, Omega, Singularity, Thrust, Motor, Collision recorded; sample cap separate | Batch/validation regressions, exported reason counts | A batch can reject multiple lanes; counts are not identical to scalar early-stop point counts |
| Quad r2 does not fix acceleration/jerk/attitude/rates at joins | `diagnose`, original physical boundary diagnostics | Known flat-continuous cubic pair has measured acceleration/attitude jump | These are diagnostics, not extra reduced-mode rejection conditions |
| First path and fixed-budget path must not be mixed | Immutable `Plan.history`, `first_solution`, `final_solution`, `incumbents`, event timestamps | C++ serialization tests and Python predecessor-checkpoint/failure tests | Old final-only files cannot reconstruct a first path |
| Native verification distinct from reduced success | `native_check.py` calls official executable only when model parameters match | Unit tests exercise command/status handling with a mock, not a native feasibility claim | Requires external checker/dependencies; model mismatch/not-requested is not pass |

The prior Assumption 1 / Definitions 2–5 / Eq.31–38 audit remains in
`theoretical_guarantees.md`. None of these measurements supplies missing
theorem constants, continuous collision guarantees, global nonlinear boundary
regularity, or automatic asymptotic optimality of standard RRTConnect.

## Native checker

The adapter uses the official `--models_base_path`, `--env_file`, and
`--result_file` options and accepts feasibility only when the executable
returns 0 and prints `OK`. Stage-specific exit status, command and logs are
kept separately. It checks the model actually supplied from the original
models directory before invocation. A mismatch is reported rather than
silently interpreting wrench controls as motor controls.

`states`, `actions`, and `times` are exported for first and final paths. Exact
continuous controls were sampled, not integrated with a ZOH controller;
native dynamics residual checks may fail, especially at reduced-r2 joins or
full physical endpoints. No native passing result is inferred from plotting,
serialization, a mocked unit test, or a matching file format.

Sources: [FLASK, Algorithm 4 and Section VII](https://arxiv.org/html/2603.16059v4),
[DynoBench](https://github.com/quimortiz/dynobench),
[Liu et al., LQMT planning (2017)](https://arxiv.org/abs/1709.05401).
The 2017 work separately discusses lower-order planning followed by refinement;
that additional refinement is deliberately not imported into this reduced-r2
FLASK experiment.
