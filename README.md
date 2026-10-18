# FLASK-RRTConnect：首解、限时结果与 cost–time 曲线

独立 C++17 / Eigen / yaml-cpp 工程，只复现 DF + 解析 LQMT BVP + RRTConnect + SIMD 验证和 Algorithm 4 shortcut，覆盖六个 DynoBench 场景。保留论文动力学参数；原始 DynoBench 环境、起终点和模型外形用于绘图和原生模型差异对照。

## Ubuntu 编译与测试

```bash
sudo apt install -y build-essential cmake libeigen3-dev libyaml-cpp-dev \
    python3 python3-numpy python3-matplotlib python3-yaml
python3 tools/configure_six.py
cmake -S . -B build-measurements -DCMAKE_BUILD_TYPE=Release -DFLASK_ENABLE_AVX2=ON
cmake --build build-measurements --parallel 2
ctest --test-dir build-measurements --output-on-failure
/usr/bin/python3 -I -m unittest discover -s tests -p test_measurements.py -v
```

CTest包含14个启用的C++/CLI测试和6个禁用的场景基准；禁用不计作通过。Python测量测试另行运行。Windows可执行文件带`.exe`；离线依赖用`FLASK_EIGEN_SOURCE`和`FLASK_YAML_SOURCE`指定。普通与AVX2模块统一Eigen 16字节ABI，不要额外启用fast-math。

## 20个不同seed，每环境总预算10秒

```bash
RUN_DIR="results/measurements_$(date +%Y%m%d_%H%M%S)"
/usr/bin/python3 -I tools/benchmark.py \
    --executable build-measurements/flask_plan \
    --output "$RUN_DIR" --budget 10 --seeds $(seq 20 39)
/usr/bin/python3 -I tools/summarize_runs.py "$RUN_DIR"
/usr/bin/python3 -I tools/plot_six.py "$RUN_DIR" --seed 20
xdg-open "$RUN_DIR/cost_time.png"
```

默认也是seed20～39，各一次，共120次；`--seeds 20`仅运行六次。输出目录必须不存在。`--repeats 20 --seeds 20`代表同一seed重复计时，不能替代独立seed统计。失败可能使runner返回2；原始记录仍保留，可继续统计/绘图。进程上限默认预算加35秒，覆盖启动/导出，和规划预算分开。

记录时刻为1/5/10/50/100/500 ms和1/5/10 s，搜索与shortcut共用预算。`--checkpoints .001 .01 .1 1 10`可显式选择。规划器提前结束后保持最后一个incumbent，不重启搜索，不伪造持续优化。因此cost–time是首解及接受的shortcut记录，不能视为渐近最优性证据。

## 输出与图

- `first_solution/`：原始RRTConnect首解的精确多项式、物理轨迹、cost、长度、执行时间与首解计算时间。
- `final_solution/`：Algorithm 4后最终路径，同一套指标；`incumbents/`保存中间接受的shortcut。
- `runs.csv`、`statistics.md/json`：首解与最终解分别统计均值、样本SD、成功率；失败不是0代价。
- `cost_time_events.csv`：每个真实发现事件的计算时间、cost、长度、执行时间及路径文件。
- `budget_metrics.csv`：每次运行在各时刻已知的最好结果；首解之前为空值。`budget_statistics.csv`按所有运行数报告成功率，cost/长度仅对当时有解者统计。
- `diagnostics.csv`：位置、速度、加速度、jerk、姿态、角速度拼接误差，以及起终点与原始完整物理状态的偏差。`rejection_counts.csv`记录失败分类，区间奇异性保留具体原因；采样预算不足单独计数。
- `paths_first.png`、`paths_final.png`（同`paths.png`）：默认seed20的六场景概览。`plots/seed_20_r01/`含每场景单独图，标出cost J、轨迹长度、首解时间和该路径可用时间，沿途绘制模型姿态。
- `cost_time.png`与每场景`*_cost_time.png`：全部seed的真实incumbent阶梯曲线、当时有解样本的均值、成功比例。条件均值的样本集合会变化，均值不保证单调。
- `manifest.json`：二进制、CMake、内核清单、源码、输入数据的哈希与平台；`model_differences.json`区分论文模型和原始模型；`native_validation.json`单列原生检查状态。

`manifest.json`捕获运行开始时的代码；可以独立重做统计/绘图，执行版本和输入哈希另记在`analysis_manifest.json`、`plot_manifest.json`。`process_seconds`是单次CLI执行加结果读取/导入的harness耗时，不能当作纯子进程或首解时间；首解时间仅使用`first_solution_ms`。

需要某个时刻的机器人轨迹图：

```bash
/usr/bin/python3 -I tools/plot_six.py "$RUN_DIR" --seed 20 --repeat 1 \
    --at-budget .001 .005 .01 .05 .1 .5 1 5 10
```

绘图从运行时保存的DynoBench环境与原始模型快照读取障碍物、起终点、尺寸和arm length，不依赖当前examples配置，也不以规划器的球覆盖冒充原始障碍物。原始模型文件及哈希在`examples/dynobench_original_models/`。静态图的姿态示意按文件尺寸绘制，不是外部CAD网格。使用系统Python的`-I`避免用户目录Matplotlib与系统mplot3d混用。

## 数学/物理契约

`J=rho*T+integral(w^T R w dt)`，其中`w`是平坦空间控制。长度是平移速度模长积分，不含yaw；执行时间T和计算时间不同。所有长度求积、诊断、文件输出与原生检查均在规划计时外执行。首解时间来自首个完整验证通过事件；不能把它和最终路径cost/长度混配。

六场景为r=2、R=I、rho=1、flat-box均匀采样。端点lift速度显式给出；旧`unicycle_sample_speed`配置已移除。range、bounds、球覆盖、100次shortcut尝试上限等为本地显式参数，不声称恢复作者私有默认值。

Quad reduced-r2只保证flat位置/速度的连接。姿态、角速度和高阶导数跳变只诊断，不新增min-jerk、min-snap、rewiring或定向搜索。Unicycle r2检查速度平方极值以检测网格间零速；其余动力学约束和Algorithm 3碰撞仍为有限采样，可能漏检薄障碍。

规划碰撞采用单球机器人和障碍物保守球覆盖；图中原始机器人尺寸不等于该简化规划几何。图中模型无碰撞外观、reduced成功和完整动力学认证是不同结果。标准FLASK-RRTConnect不自动具有渐近最优性。

## 可选原生DynoBench checker

```bash
/usr/bin/python3 -I tools/benchmark.py \
    --executable build-measurements/flask_plan --output results/native_check_run \
    --seeds 20 --budget 10 \
    --native-checker /absolute/path/dynobench/build/check_trajectory \
    --native-models /absolute/path/dynobench/models
```

仅调用真实官方checker，不以内置近似冒充。论文Unicycle和Quad3D模型与原始模型存在动力学差异时报告`model_mismatch`，不会混用模型后声称通过。Quad2D可按匹配模型执行检查，仍可能因完整状态边界、r2拼接、有限采样、ZOH离散传播而失败。导出的states/actions有真实times，末步可能短于dt；不是动力学积分生成的认证轨迹。未提供checker时明确标记`not_requested`，不得当作通过。

## Scalar/SIMD同条件比较

相同配置、seed、预算可以分别用`--backend scalar`和`--backend simd`另存结果；实际后端写入每条记录。禁用AVX2的CPU明确使用scalar。以数值差分测试为正确性依据；计时比较需报告成功率和路径是否一致，不把失败或不同路径的耗时直接当成纯SIMD加速。

详细测量契约与Theory-to-Code Matrix见[measurement_protocol.md](docs/measurement_protocol.md)、[theoretical_guarantees.md](docs/theoretical_guarantees.md)。本轮120次本机运行与独立积分审计见[measurement_results_20261018.md](docs/measurement_results_20261018.md)，可用`python3 tools/audit_measurements.py "$RUN_DIR"`复核保存路径。历史实验和历史构建保留，旧版本的最终路径不能事后恢复为真实首解；新统计需要result_version=2数据。
