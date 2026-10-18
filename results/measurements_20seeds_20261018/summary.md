# FLASK-RRTConnect first / final / budget measurements

Seeds: [20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39]; repeats per seed: 1; total planning budget: 10.0 s.
J = rho*T + integral(w^T R w dt); execution duration is not planning time.
Means +/- sample SD are over successful runs only. Before a first solution, cost and length are missing.
First metrics belong to the saved raw RRTConnect path; final metrics belong to the shortcut path.
Search and shortcut share the budget. Export, quadrature and native checks are outside the measured planning clock.

## First solution

| Environment | Success | First time (ms) | First J | First length (m) | First execution duration (s) |
| --- | --- | --- | --- | --- | --- |
| uni_bugtrap | 1/20 | 731.325 +/- NA | 60.491 +/- NA | 34.065 +/- NA | 47.390 +/- NA |
| uni_wall | 20/20 | 456.498 +/- 938.622 | 34.744 +/- 11.099 | 18.808 +/- 6.203 | 27.215 +/- 8.777 |
| quad2d_bugtrap | 18/20 | 3.814 +/- 3.578 | 36.287 +/- 11.495 | 16.705 +/- 4.211 | 21.789 +/- 6.625 |
| quad2d_hole | 19/20 | 2.126 +/- 2.026 | 23.124 +/- 10.216 | 10.099 +/- 4.160 | 14.195 +/- 6.048 |
| quad3d_block | 20/20 | 5.768 +/- 3.360 | 18.588 +/- 5.930 | 11.083 +/- 2.506 | 12.081 +/- 3.114 |
| quad3d_window | 20/20 | 4.947 +/- 4.149 | 26.074 +/- 12.658 | 10.591 +/- 4.209 | 15.732 +/- 6.341 |

## Final solution

| Environment | Success | Final J | Final length (m) | Execution duration (s) | Planning incl. shortcut (ms) | Last improvement (ms) |
| --- | --- | --- | --- | --- | --- | --- |
| uni_bugtrap | 1/20 | 60.491 +/- NA | 34.065 +/- NA | 47.390 +/- NA | 733.528 +/- NA | 731.325 +/- NA |
| uni_wall | 20/20 | 28.609 +/- 13.328 | 15.723 +/- 7.389 | 22.194 +/- 10.718 | 459.976 +/- 938.732 | 459.112 +/- 937.924 |
| quad2d_bugtrap | 18/20 | 21.649 +/- 4.880 | 12.531 +/- 1.764 | 13.927 +/- 1.943 | 4.590 +/- 3.571 | 4.424 +/- 3.592 |
| quad2d_hole | 19/20 | 13.501 +/- 3.002 | 6.758 +/- 1.512 | 9.007 +/- 2.065 | 2.496 +/- 2.322 | 2.445 +/- 2.236 |
| quad3d_block | 20/20 | 13.217 +/- 1.926 | 9.048 +/- 0.937 | 9.432 +/- 1.180 | 8.153 +/- 3.712 | 8.097 +/- 3.637 |
| quad3d_window | 20/20 | 12.010 +/- 2.833 | 6.409 +/- 0.964 | 8.435 +/- 1.600 | 5.647 +/- 4.377 | 5.632 +/- 4.389 |

## Checkpoints

| Environment | Budget (s) | Success | Mean J (available runs) | Mean length (m) |
| --- | --- | --- | --- | --- |
| dynobench_uni_bugtrap | 0.001 | 0/20 | NA | NA |
| dynobench_uni_bugtrap | 0.005 | 0/20 | NA | NA |
| dynobench_uni_bugtrap | 0.01 | 0/20 | NA | NA |
| dynobench_uni_bugtrap | 0.05 | 0/20 | NA | NA |
| dynobench_uni_bugtrap | 0.1 | 0/20 | NA | NA |
| dynobench_uni_bugtrap | 0.5 | 0/20 | NA | NA |
| dynobench_uni_bugtrap | 1 | 1/20 | 60.4911 | 34.0648 |
| dynobench_uni_bugtrap | 5 | 1/20 | 60.4911 | 34.0648 |
| dynobench_uni_bugtrap | 10 | 1/20 | 60.4911 | 34.0648 |
| dynobench_uni_wall | 0.001 | 0/20 | NA | NA |
| dynobench_uni_wall | 0.005 | 0/20 | NA | NA |
| dynobench_uni_wall | 0.01 | 0/20 | NA | NA |
| dynobench_uni_wall | 0.05 | 1/20 | 19.4321 | 11.3045 |
| dynobench_uni_wall | 0.1 | 5/20 | 24.0669 | 13.6694 |
| dynobench_uni_wall | 0.5 | 16/20 | 25.6344 | 14.2785 |
| dynobench_uni_wall | 1 | 19/20 | 26.4587 | 14.5941 |
| dynobench_uni_wall | 5 | 20/20 | 28.6088 | 15.7227 |
| dynobench_uni_wall | 10 | 20/20 | 28.6088 | 15.7227 |
| dynobench_quad2d_bugtrap | 0.001 | 1/20 | 31.2923 | 14.8886 |
| dynobench_quad2d_bugtrap | 0.005 | 16/20 | 23.5302 | 13.0405 |
| dynobench_quad2d_bugtrap | 0.01 | 17/20 | 23.4303 | 13.0714 |
| dynobench_quad2d_bugtrap | 0.05 | 18/20 | 21.6492 | 12.5306 |
| dynobench_quad2d_bugtrap | 0.1 | 18/20 | 21.6492 | 12.5306 |
| dynobench_quad2d_bugtrap | 0.5 | 18/20 | 21.6492 | 12.5306 |
| dynobench_quad2d_bugtrap | 1 | 18/20 | 21.6492 | 12.5306 |
| dynobench_quad2d_bugtrap | 5 | 18/20 | 21.6492 | 12.5306 |
| dynobench_quad2d_bugtrap | 10 | 18/20 | 21.6492 | 12.5306 |
| dynobench_quad2d_hole | 0.001 | 7/20 | 14.0604 | 6.59412 |
| dynobench_quad2d_hole | 0.005 | 17/20 | 12.9143 | 6.52057 |
| dynobench_quad2d_hole | 0.01 | 19/20 | 13.5006 | 6.7579 |
| dynobench_quad2d_hole | 0.05 | 19/20 | 13.5006 | 6.7579 |
| dynobench_quad2d_hole | 0.1 | 19/20 | 13.5006 | 6.7579 |
| dynobench_quad2d_hole | 0.5 | 19/20 | 13.5006 | 6.7579 |
| dynobench_quad2d_hole | 1 | 19/20 | 13.5006 | 6.7579 |
| dynobench_quad2d_hole | 5 | 19/20 | 13.5006 | 6.7579 |
| dynobench_quad2d_hole | 10 | 19/20 | 13.5006 | 6.7579 |
| dynobench_quad3d_block | 0.001 | 0/20 | NA | NA |
| dynobench_quad3d_block | 0.005 | 12/20 | 14.4509 | 9.56882 |
| dynobench_quad3d_block | 0.01 | 16/20 | 13.3894 | 9.05816 |
| dynobench_quad3d_block | 0.05 | 20/20 | 13.2167 | 9.04783 |
| dynobench_quad3d_block | 0.1 | 20/20 | 13.2167 | 9.04783 |
| dynobench_quad3d_block | 0.5 | 20/20 | 13.2167 | 9.04783 |
| dynobench_quad3d_block | 1 | 20/20 | 13.2167 | 9.04783 |
| dynobench_quad3d_block | 5 | 20/20 | 13.2167 | 9.04783 |
| dynobench_quad3d_block | 10 | 20/20 | 13.2167 | 9.04783 |
| dynobench_quad3d_window | 0.001 | 2/20 | 9.15931 | 5.23979 |
| dynobench_quad3d_window | 0.005 | 12/20 | 11.5248 | 6.15242 |
| dynobench_quad3d_window | 0.01 | 18/20 | 11.5529 | 6.26074 |
| dynobench_quad3d_window | 0.05 | 20/20 | 12.0098 | 6.40917 |
| dynobench_quad3d_window | 0.1 | 20/20 | 12.0098 | 6.40917 |
| dynobench_quad3d_window | 0.5 | 20/20 | 12.0098 | 6.40917 |
| dynobench_quad3d_window | 1 | 20/20 | 12.0098 | 6.40917 |
| dynobench_quad3d_window | 5 | 20/20 | 12.0098 | 6.40917 |
| dynobench_quad3d_window | 10 | 20/20 | 12.0098 | 6.40917 |

A changing available-run cohort can change the conditional mean; it is not an unconditional optimality claim.
RRTConnect is not restarted after termination. Later checkpoints hold its final incumbent.
Paper dynamics are retained; model_differences.json records differences from original DynoBench models.
Reduced-r2 success is separate from native dynamics certification. See native_validation.json.

- dynobench_uni_bugtrap: {'timeout': 11, 'iteration_limit': 8}; failed planning time: 9619.788 +/- 524.006 ms
- dynobench_quad2d_bugtrap: {'node_limit': 2}; failed planning time: 2742.485 +/- 206.442 ms
- dynobench_quad2d_hole: {'node_limit': 1}; failed planning time: 3261.070 +/- NA ms
