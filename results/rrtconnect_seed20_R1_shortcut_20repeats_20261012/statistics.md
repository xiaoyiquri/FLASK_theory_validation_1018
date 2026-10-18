# FLASK-RRTConnect repeated-run statistics

Seeds: [20]; repeats per seed/environment: 20.
Each repetition launches a fresh process; there are no warm-up exclusions.
Metrics below are mean +/- sample SD over successful runs only.
First-solution time excludes shortcut; total planning time includes shortcut.
Cost and length belong to the final path after shortcut. I/O is excluded.

| Environment | Success | First solution (ms) | Total planning (ms) | Cost | Length (m) |
| --- | --- | --- | --- | --- | --- |
| uni_bugtrap | 0/20 | NA | NA | NA | NA |
| uni_wall | 20/20 | 1906.383 +/- 137.942 | 1923.113 +/- 140.149 | 48.117 +/- 0.000 | 25.516 +/- 0.000 |
| quad2d_bugtrap | 20/20 | 26.548 +/- 4.661 | 36.022 +/- 5.820 | 22.809 +/- 0.000 | 14.099 +/- 0.000 |
| quad2d_hole | 20/20 | 5.431 +/- 1.028 | 7.151 +/- 1.185 | 11.040 +/- 0.000 | 5.647 +/- 0.000 |
| quad3d_block | 20/20 | 9.295 +/- 1.798 | 14.583 +/- 2.629 | 12.711 +/- 0.000 | 9.278 +/- 0.000 |
| quad3d_window | 20/20 | 36.251 +/- 5.724 | 40.512 +/- 6.154 | 13.031 +/- 0.000 | 6.487 +/- 0.000 |

Failure latency is reported separately; a timeout is not a measured first solution.
- dynobench_uni_bugtrap: {'timeout': 20}; failed planning latency (ms): 10000.090 +/- 0.323.

With one fixed seed, repetitions measure timing variability for the same sampling sequence.
They do not measure success probability over independently sampled seeds.
These are local DynoBench-data runs; reduced-r2 quad paths have no official dynamics certification.
