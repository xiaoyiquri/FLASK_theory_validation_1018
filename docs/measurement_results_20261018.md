# 2026-10-18 本机测量与交付说明

本轮保留论文动力学参数，六场景均为 r=2、R=I、rho=1、开启
Algorithm 4 shortcut。每环境 seed=20～39 各运行一次，总计120次；
搜索和shortcut共享10秒上限，并保留原有迭代数和节点数上限。
这是本机 Windows x86-64 Release 测量，不能替代用户 Ubuntu 虚拟机计时，
也不能据此声称达到论文表格中的性能。

| 环境 | 成功 | 首解时间均值(ms) | 首解J均值 | 首解长度均值(m) | 10秒时J均值 | 10秒时长度均值(m) |
| --- | --- | --- | --- | --- | --- | --- |
| Unicycle Bugtrap | 1/20 | 731.325 | 60.491 | 34.065 | 60.491 | 34.065 |
| Unicycle Wall | 20/20 | 456.498 | 34.744 | 18.808 | 28.609 | 15.723 |
| Quad2D Bugtrap | 18/20 | 3.814 | 36.287 | 16.705 | 21.649 | 12.531 |
| Quad2D Hole | 19/20 | 2.126 | 23.124 | 10.099 | 13.501 | 6.758 |
| Quad3D Block | 20/20 | 5.768 | 18.588 | 11.083 | 13.217 | 9.048 |
| Quad3D Window | 20/20 | 4.947 | 26.074 | 10.591 | 12.010 | 6.409 |

均值仅使用有解运行，失败没有用0补齐。Bugtrap只有一个成功样本，
不能估计样本标准差。标准差和全部9个时刻的成功率、cost、长度见
`results/measurements_20seeds_20261018/statistics.md`、`budget_statistics.csv`。
Unicycle Bugtrap失败为11次timeout、8次iteration_limit；Quad2D两场的
3次失败均为node_limit。没有CLI加载错误。未调整采样、约束或资源上限来
掩盖这些失败；Unicycle Bugtrap成功率仍是与论文结果之间的明显差距。

cost–time图中蓝色细线是各运行实际已知路径，红线是当时有解运行的
条件均值，绿色虚线/右轴是全部20次运行中的成功比例。新运行加入
有解集合可能使条件均值上升。标准RRTConnect找到首解并完成一次
shortcut后会停止，后续时刻保持最后路径，不表示仍在优化或具有渐近最优性。

## 路径、模型和验证范围

- `paths_first.png`和`paths_final.png`使用同一seed=20，分别展示原始首解与shortcut结果。
  该seed的Unicycle Bugtrap失败，图中明确标为NA。
  `plots/seed_27_r01/`另有唯一成功Bugtrap的首解/最终路径示例，
  明确标注seed=27和成功1/20，不替换seed=20概览或全部seed统计。
- 原始DynoBench环境文件提供障碍物、起终点；原始模型文件提供外形尺寸。
  规划保留论文模型；具体差异见`model_differences.json`。
  原生外形绘图和规划碰撞中的球近似不是同一几何契约。
- Quad reduced-r2拼接的姿态/角速度/高阶导数不连续和完整物理边界误差
  单独写入`diagnostics.csv`，没有用额外轨迹优化方法隐去这些问题。
  有限碰撞采样可能漏检薄障碍，未获得连续轨迹有效性证书。
- 真实官方DynoBench checker本轮未运行，`native_verified_stages=0`。
  原生适配单元测试使用mock，只验证调用与状态分类。

## 已完成的检查

启用的14项C++/CLI CTest全部通过；6项场景基准仍禁用，不能计为通过。
9项Python测量、模型快照、姿态和原生适配测试通过。
独立审计覆盖120次运行、1080条预算记录，以及196条首解/最终路径：

| 独立审计项目 | 结果 |
| --- | --- |
| Gauss求积复算LQMT目标的最大相对误差 | 8.184e-16 |
| 高阶求积与导出长度的最大差值 | 0.0001141 m |
| flat端点/拼接状态最大误差 | 3.553e-15 |
| 成功路径数 / 原生认证阶段数 | 196 / 0 |

相同Quad3D Window、seed=20的scalar/SIMD复核中，首解和最终路径的
多项式系数、cost、长度差值均为0。这是单次功能一致性证据，不是普遍加速比结论。

运行开始时的源码/二进制/输入哈希保存在`manifest.json`；独立统计和
绘图所用版本分别保存在`analysis_manifest.json`和`plot_manifest.json`。
详细审计见`measurement_audit.json`、`scalar_simd_check.json`。

## Ubuntu 重跑

将本次完整源码包解压到工程根目录，用新的构建目录按README编译。
安装新增绘图依赖`python3-yaml`和`python3-numpy`；已有依赖可保留。
使用`/usr/bin/python3 -I`隔离此前用户目录与系统Matplotlib混用问题。
不要混用旧可执行文件与version-2脚本；旧final-only结果无法恢复真实首解。

```bash
RUN_DIR="results/measurements_$(date +%Y%m%d_%H%M%S)"
/usr/bin/python3 -I tools/benchmark.py \
    --executable build-measurements/flask_plan \
    --output "$RUN_DIR" --budget 10 --seeds $(seq 20 39)
/usr/bin/python3 -I tools/summarize_runs.py "$RUN_DIR"
/usr/bin/python3 -I tools/plot_six.py "$RUN_DIR" --seed 20 \
    --at-budget .001 .005 .01 .05 .1 .5 1 5 10
/usr/bin/python3 -I tools/audit_measurements.py "$RUN_DIR"
xdg-open "$RUN_DIR/cost_time.png"
```
