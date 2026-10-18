# Batch validation repair — 2026-10-18

The reported Ubuntu Release build succeeded, but 7 of the 14 enabled CTest
tests failed. Six DynoBench benchmark entries were explicitly disabled, rather
than passing. These failures do not indicate missing package dependencies.

## Causes and changes

1. Both the validator and batch differential test constructed times as
   `(T*i)/intervals`. Floating-point multiplication followed by division can
   put the endpoint just above `T`, causing `Invalid batch time`. For example,
   `T=0.6000000000000001` produces `0.6000000000000002` at `(T*7)/7`.
   Construct
   `T*(double(i)/double(intervals))` instead, identically for SIMD and scalar
   grids. A positive subnormal duration also keeps at least one interval and
   two endpoint samples. The API still rejects negative, nonfinite and
   out-of-range active times; inactive NaN lanes remain ignored.
2. `BatchContext` contains fixed-size Eigen matrices. The baseline and AVX2
   translation units previously used different default Eigen alignments. A
   local x86-64 Clang layout probe found context sizes 736/768 and validity
   flag offsets 720/736, respectively. These are measurements for that
   toolchain, not portable layout constants. The AVX2 kernel could read the
   wrong fields, rejecting feasible curves or applying the wrong allocation.
   `flask_runtime` now exports `EIGEN_MAX_ALIGN_BYTES=16` and
   `EIGEN_MAX_STATIC_ALIGN_BYTES=16` to all its consumers. Explicit AVX2 loads
   remain unaligned and AVX2 is still isolated to dispatched source files.

Eigen documents this use of a common alignment to maintain SSE/AVX ABI
compatibility: [Eigen preprocessor directives](https://eigen.tuxfamily.org/dox/TopicPreprocessorDirectives.html).

No physical limits, obstacles, BVP objective, planner seed, shortcut acceptance
rule or test success condition is relaxed. Scalar evaluation remains an
explicit reference and the supported-CPU dispatch remains enabled.

## Rebuild on Ubuntu

After copying the repair files into the project root, use a fresh build
directory to ensure every translation unit sees the same Eigen definitions:

```bash
cmake -S . -B build-linux-fix1018 -DCMAKE_BUILD_TYPE=Release -DFLASK_ENABLE_AVX2=ON
cmake --build build-linux-fix1018 --parallel 2
ctest --test-dir build-linux-fix1018 --output-on-failure
```

Run benchmarks only after the enabled tests pass. The original six disabled
entries are benchmark cases; run them with `tools/benchmark.py` without treating
an individual planning timeout as a unit-test assertion.

## Verification

Local Windows x86-64 Clang 18.1.6 Release build completed successfully.
CTest passed all **14 enabled tests**, including all 7 previously reported
failures. The 6 DynoBench benchmark entries remain explicitly disabled and
are not counted as passing tests. The scalar/SIMD batch differential test,
thin-obstacle regression, nonlinear model reconstruction tests, endpoint-grid
regressions and all 3 CLI smoke tests passed. Frozen vendor header hashes
(16 files) and the historical executable hash were preserved.

This is local verification, not a claim that the user's Ubuntu virtual
machine has already been retested. Copy the updated files there and run the
commands above. No new six-environment timing or success-rate experiment was
run as part of this repair; no official DynoBench dynamics certification is
claimed.
