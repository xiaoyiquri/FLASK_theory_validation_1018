# 本轮实现与验证：2026-10-12

> 历史报告：本文件描述本轮收窄任务以前的full-r4/传播版本。当前FLASK-RRTConnect六场景r2版本及18/18结果见 [rrtconnect_six_review.md](rrtconnect_six_review.md)。下述旧数据保留，不代表当前版本。

本轮把独立理论项目补成可执行规划工程。实现清单见 [paper_implementation_status.md](paper_implementation_status.md)，理论条件与例外见 [theoretical_guarantees.md](theoretical_guarantees.md)。

## 构建环境与复现命令

Windows x86_64，Zig 0.13.0 / Clang 18.1.6，Eigen 3.4.0，yaml-cpp 0.8.0，CMake 4.4.3，Ninja，C++17，Release `-O3 -DNDEBUG`，禁用 FMA contraction / fast-math。AVX2 在本机可用；指令仅编译到单独源文件，运行时同时检查 CPU 和 OS YMM 支持。

从 `D:/codex_talk/project_1` 执行：

```powershell
$env:ZIG_GLOBAL_CACHE_DIR='D:/codex_talk/project_1/tmp/zig-cache'
$env:ZIG_LOCAL_CACHE_DIR='D:/codex_talk/project_1/tmp/flask-zig-local'
& './tmp/simd-build-tools/cmake/data/bin/cmake.exe' -S project_code/FLASK_theory_validation -B tmp/build_flask_runtime -G Ninja '-DCMAKE_MAKE_PROGRAM=D:/codex_talk/project_1/tmp/simd-build-tools/bin/ninja.exe' '-DCMAKE_TOOLCHAIN_FILE=D:/codex_talk/project_1/tmp/zig-toolchain.cmake' '-DFLASK_EIGEN_SOURCE=D:/codex_talk/project_1/tmp/eigen-3.4.0' '-DFLASK_YAML_SOURCE=D:/codex_talk/project_1/tmp/yaml-cpp-0.8.0' -DCMAKE_BUILD_TYPE=Release
& './tmp/simd-build-tools/cmake/data/bin/cmake.exe' --build tmp/build_flask_runtime --parallel 4
& './tmp/simd-build-tools/cmake/data/bin/ctest.exe' --test-dir tmp/build_flask_runtime --output-on-failure
```

通用构建不需要这些本机工具路径，按 README 即可。未下载依赖，未修改旧 RRT*/PFF 工程、原 DynoBench 数据或既有实验结果。

## 测试结果

最终 **14/14 CTest 通过**，总计3.55秒。该时间是测试套件运行时间，不能当作规划加速比。
原始输出见 [runtime_ctest.log](runtime_ctest.log)，早期输出另保存在 [runtime_initial_ctest.log](runtime_initial_ctest.log)。

| 测试 | 实际检查 |
| --- | --- |
| controllability / custom | 原48组动态n/r、9组实际模板、额外5/7、截断rank/各输入指数与退化负例 |
| gramian_scaling / custom | 原独立积分、det/logdet、r²n指数及高阶数值范围 |
| theory_assumptions | 非零/零速、角度接缝、Quad reduced非唯一性、控制导数阶数、理论尺度与薄障碍反例 |
| bvp | 完整端点、固定/最优时间、rho、r2四次条件、多正根成本选优、独立成本积分、r4/r5链动力学 |
| planner | 实际绕障、正时间双树拼接、固定seed重现、输入碰撞拒绝、成本不增加；对照作者公开port的extent metric与BVP插值 |
| validation | 真实有限验证器漏检薄障碍，Bernstein几何细分拒绝；normalized-time曲线、inactive lane/剩余坐标初始化、样本上限、scalar/AVX2 mask一致 |
| models | Unicycle r2/r3、Quad2D/Quad3D r4/r5的独立原始非线性ODE残差；默认拒绝reduced物理拼接跳变；sampling bounds与物理约束分离 |
| propagation | Eq.29解析传播；BestNear/witness/重启阶段与DP开关；BVP目标捷径与纯传播goal region |
| io | 普通YAML到精确曲线/物理样本；DynoBench格式导入与数据快照；尺寸错误及速度分支不一致拒绝 |
| CLI 3项 | Integrator绕障、Unicycle直线、Quad3D full-r4小位移，均成功 |

最终单位测试中的实际绕障为4节点、成本9.92525；它只证明该fixture路径成功，没有基于它推导真实六场景性能。
所有测试使用抛异常的 `require`，不会因为Release定义NDEBUG而被移除。

## 六个真实数据场景

场景来自本地 DynoBench，独立快照、来源与哈希见 `examples/dynobench_data/SOURCE_MANIFEST.json`。Unicycle/Quad3D模型使用原工程记录的paper profile，Quad2D使用DynoBench模型。所有球半径、球化cell、derivative lifting、full-r4边界、停止预算和seed明确在配置中；不是作者默认配置证书。

先做10000次迭代上限的有限烟雾检查，发现程序会在0.05–0.3秒达到迭代上限，明显未用满3秒时间预算。因而另建立 `examples/time_budget`：仍为3秒、seed41、相同几何和采样域，但将迭代上限提升到1000000，节点上限仍为10000。两种预算与所有旧结果分别保留，不选择性覆盖。

最终源码采用公开OMPL移植版的BVP插值机制。该代码的time-budget配置实测如下，规划时间包含已执行的shortcut；进程启动时间另存在CSV中。

| 场景 | seed | 结果 | 原因 | 节点数 | 规划时间(s) | 成功成本 |
| --- | --- | --- | --- | --- | --- | --- |
| Unicycle Bugtrap | 41 | 失败 | timeout | 267 | 3.000000 | NA |
| Unicycle Wall | 41 | 成功 | connected | 194 | 0.146391 | 37.9287 |
| Quad2D Bugtrap | 41 | 成功 | connected | 56 | 0.771238 | 28.8128 |
| Quad2D Hole | 41 | 成功 | connected | 325 | 2.572530 | 24.5203 |
| Quad3D Window | 41 | 失败 | timeout | 36 | 3.000020 | NA |
| Quad3D Block | 41 | 成功 | connected | 20 | 0.394718 | 21.1772 |

表中时间/成本来自CLI输出；结果YAML保存17位精度的值。失败成本/时长为null，CSV成本空白，均值只计算成功运行；每场仅一seed，样本标准差为NA。两个超时没有再次加时重跑。

逐项CSV、stdout日志、成功的精确轨迹/物理样本与失败原因在 [results/time_budget_20261012](../results/time_budget_20261012/runs.csv)。历史轮次：

- `results/dynobench_smoke_20261012`：初版10000迭代，暴露误将采样边界作为额外物理约束，0/6。保留作诊断，不用作性能比较。
- `results/dynobench_recheck_20261012`：边界修正，原线性flat插值，1/6；之后根据公开作者代码修正插值，不再是当前行为。
- `results/ompl_policy_20261012`：最终BVP插值与extent metric，10000迭代，0/6，全部提前触发迭代上限。
- `results/time_budget_20261012`：同一最终源码，用满最多3秒预算，4/6。

批量命令（输出目录需要不存在）：

```powershell
python project_code/FLASK_theory_validation/tools/benchmark.py --executable tmp/build_flask_runtime/flask_plan.exe --scenarios project_code/FLASK_theory_validation/examples/time_budget/dynobench_uni_bugtrap.yaml project_code/FLASK_theory_validation/examples/time_budget/dynobench_uni_wall.yaml project_code/FLASK_theory_validation/examples/time_budget/dynobench_quad2d_bugtrap.yaml project_code/FLASK_theory_validation/examples/time_budget/dynobench_quad2d_hole.yaml project_code/FLASK_theory_validation/examples/time_budget/dynobench_quad3d_window.yaml project_code/FLASK_theory_validation/examples/time_budget/dynobench_quad3d_block.yaml --seeds 41 --process-timeout 6 --output project_code/FLASK_theory_validation/results/new_time_budget_run
```

这里没有原生DynoBench checker、FCL、作者私有VAMP流水线或iDb-A*/SST*/Kino-PAX对照，也没有20/100次统计。连续sphere检查只覆盖几何，控制/状态/奇异性仍为有限采样。本轮成功不能等同论文的完整保证或表格结果。

## 公开作者源码核对

核对 [OMPL PR #1472](https://github.com/ompl/ompl/pull/1472)，提交 `fe44ef0448e098988772df074507e9f143570e83`。该PR作者区分了正在公开移植的OMPL版本与原私有VAMP实现。
源码快照见 [references/ompl_fe44ef0/SOURCE.md](../references/ompl_fe44ef0/SOURCE.md)。本项目据其欧氏metric/interpolation修正搜索机制，没有把完整OMPL/VAMP集成或私有实验参数写成已经复现。

当前仍未实现公开port的SO(2)/compound chart、trajectory-cost NN、OMPL/Python绑定；未实现整条float8物理映射/FK流水线、真实机械臂模型/硬件实验、完整控制约束连续证书，以及两项超时场景在论文预算下的性能。这些明确列为剩余工作，不由理论CTest通过来豁免。
