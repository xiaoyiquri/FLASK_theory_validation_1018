# 验证记录

2026-10-11，Windows x86_64，Zig 0.13.0 的 Clang 18.1.6，Eigen 3.4.0，CMake 4.4.3，C++17。
仅本独立项目被构建，参考工程 header 被只读使用。
没有运行真实 DynoBench/FCL 或完整规划器，没有概率常数/性能复现结论。

从 `D:/codex_talk/project_1` 执行的 PowerShell 命令：

```powershell
$env:ZIG_GLOBAL_CACHE_DIR='D:/codex_talk/project_1/tmp/zig-cache'
$env:ZIG_LOCAL_CACHE_DIR='D:/codex_talk/project_1/tmp/flask-zig-local'
& './tmp/simd-build-tools/cmake/data/bin/cmake.exe' -S project_code/FLASK_theory_validation -B tmp/build_flask_theory -G Ninja '-DCMAKE_MAKE_PROGRAM=D:/codex_talk/project_1/tmp/simd-build-tools/bin/ninja.exe' '-DCMAKE_TOOLCHAIN_FILE=D:/codex_talk/project_1/tmp/zig-toolchain.cmake' '-DFLASK_EIGEN_SOURCE=D:/codex_talk/project_1/tmp/eigen-3.4.0' -DCMAKE_BUILD_TYPE=Release
& './tmp/simd-build-tools/cmake/data/bin/cmake.exe' --build tmp/build_flask_theory -j 2
& './tmp/simd-build-tools/cmake/data/bin/ctest.exe' --test-dir tmp/build_flask_theory --output-on-failure
```

工具缓存写入工作区，避免编译器默认缓存位置不可写。通用工具链可按 README 构建。
本机初次配置识别编译器前的缓存导致 `CMAKE_CXX_FLAGS_RELEASE` 为空；本次编译没有优化标志，因此以下耗时只用于测试运行记录，不用于任何性能比较。
编译时唯一警告来自参考 `quad3d_df.h` 已有的未使用 metadata 参数，未修改该 header。

| CTest | 结果 | 检查内容 |
| --- | --- | --- |
| controllability | PASS | 48 组动态维度、9 个实际模板内核、截断 rank、各输入指数和缺失输入负例 |
| gramian_scaling | PASS | 同维度扫描，6 个小时间，带交叉项的 SPD 权重，独立 Gauss 积分、det/logdet、斜率和奇异权重负例 |
| theory_assumptions | PASS | 实际 Unicycle/Quad3D 映射正例和反例，符号 Eq.33 尺度、Eq.31 距离语义、薄障碍采样漏检 |
| controllability_custom | PASS | n=5,r=7，rank=35，各输入指数7 |
| gramian_scaling_custom | PASS | n=5,r=7，理论指数245，实测斜率245.000000004，log(C)=-448.819349478 |

5/5 通过，总运行时间 10.50 秒。原始 CTest 输出保存在 [validation_ctest.log](validation_ctest.log)。
反例测试的 PASS 表示成功发现违反前提的情况，不表示反例模型满足定理。

参考代码 SHA-256（相对于 `RRTstarDFPFF_paper_r2_float32_strict_20261010`）：

```text
pff/pff_integrator.h 1a505cd5b2eedbc25aec18056ad0a0b6f73de61cf0fb42a3030ca8d5b9983de5
common/flat_state.h 898027fb1927acda51c827a9ab9cde52be1af5a7411f034c283babafc688a1e7
common/flat_trajectory.h 1ac9b85db43017d152595ff441b2b80740099782a6562f3a51664468a76833f2
df/unicycle_df.h c5b764ef68fe75fd7f474caebe709b7c733ab4ebd026a8a523bd4c252c7cf34b
df/paper_r2_df.h e5d754ccdd3c5d2747958097ad4080b93b59ca41aff909c0882c923b4bd744da
df/quad3d_df.h 1d8401934e19ac09d2cf3c889d566fc818924eae37f99e55489cb3ae011e16b7
core/nearest_neighbor.h 04da918cdb83a9c0d37615d84adfed3cf1f3c46366affa73b9b1853c52bfeb37
core/rrtstar.h f3e3d32f3e2eea4d6a5e47782900f7b48c470d9efd2c719321afb26694ef17a3
systems/dyno_model.h 62a852fc912090bc6cb505dbbcf3b5637de154e9bf80bf7502cbae03fada277c
simd/discrete_validator.h d3014c0008cbe91147cc06f3fffee71d2e66dee864413b172119d1a0b054b1f5
```
