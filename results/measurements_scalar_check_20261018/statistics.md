# FLASK-RRTConnect first / final / budget measurements

Seeds: [20]; repeats per seed: 1; total planning budget: 10.0 s.
J = rho*T + integral(w^T R w dt); execution duration is not planning time.
Means +/- sample SD are over successful runs only. Before a first solution, cost and length are missing.
First metrics belong to the saved raw RRTConnect path; final metrics belong to the shortcut path.
Search and shortcut share the budget. Export, quadrature and native checks are outside the measured planning clock.

## First solution

| Environment | Success | First time (ms) | First J | First length (m) | First execution duration (s) |
| --- | --- | --- | --- | --- | --- |
| quad3d_window | 1/1 | 28.396 +/- NA | 36.448 +/- NA | 15.266 +/- NA | 21.655 +/- NA |

## Final solution

| Environment | Success | Final J | Final length (m) | Execution duration (s) | Planning incl. shortcut (ms) | Last improvement (ms) |
| --- | --- | --- | --- | --- | --- | --- |
| quad3d_window | 1/1 | 10.903 +/- NA | 6.264 +/- NA | 8.041 +/- NA | 29.750 +/- NA | 29.722 +/- NA |

## Checkpoints

| Environment | Budget (s) | Success | Mean J (available runs) | Mean length (m) |
| --- | --- | --- | --- | --- |
| dynobench_quad3d_window | 0.001 | 0/1 | NA | NA |
| dynobench_quad3d_window | 0.005 | 0/1 | NA | NA |
| dynobench_quad3d_window | 0.01 | 0/1 | NA | NA |
| dynobench_quad3d_window | 0.05 | 1/1 | 10.9032 | 6.26419 |
| dynobench_quad3d_window | 0.1 | 1/1 | 10.9032 | 6.26419 |
| dynobench_quad3d_window | 0.5 | 1/1 | 10.9032 | 6.26419 |
| dynobench_quad3d_window | 1 | 1/1 | 10.9032 | 6.26419 |
| dynobench_quad3d_window | 5 | 1/1 | 10.9032 | 6.26419 |
| dynobench_quad3d_window | 10 | 1/1 | 10.9032 | 6.26419 |

A changing available-run cohort can change the conditional mean; it is not an unconditional optimality claim.
RRTConnect is not restarted after termination. Later checkpoints hold its final incumbent.
Paper dynamics are retained; model_differences.json records differences from original DynoBench models.
Reduced-r2 success is separate from native dynamics certification. See native_validation.json.

