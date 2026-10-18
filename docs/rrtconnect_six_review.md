# FLASK-RRTConnect 六场景重新审查（2026-10-12）

当前范围仅截图第一列：DF + RRTConnect，在六份 DynoBench 场景数据上运行独立规划器。不是RRT*、SST或DP-RRT；本轮已移除额外传播搜索代码和相应测试，原理论审计保留。

## 审查与修正

1. 六场景使用practical r2；quad默认full-r4与高阶端点强制约束偏离当前目标，改为明确的reduced契约。逐边仍从cubic完整jet恢复原状态/控制并检查样本限幅，记录真实高阶端点/拼接误差。
2. 双向树按正时间保存边。start扩展取outgoing BVP前段，goal扩展取target→parent的incoming BVP末段。这是本地kinodynamic适配，区别于公开OMPL几何port双向同插值，不能冒称作者私有实现默认。
3. 原始随机目标可以碰撞，只检查range截短后的实际节点/边；避免提前拒绝合法的部分扩展。
4. Unicycle r2补速度模平方所有极值检查，拒绝网格间零速/速度超限；omega和其他控制仍是有限采样。新增网格漏检零速负例（t=.37）和normalized系数测试。
5. 六场景Unicycle选择pose均匀heading+0.5m/s lift、R=10I；quad使用flat box采样、R=I；rho=1。R=10I是Eq.20允许的本地SPD权重，改善本地转速受限候选的可接受性；并未恢复作者未公开参数，也未放宽v/omega限幅。
6. 单独积分平移速度得到米制长度；cost不再冒充Traj. Len.。planning time包含shortcut，排除配置加载、长度求积、进程启动和I/O；搜索时间另列。失败保留为空/Null而不是0。
7. 输出精确曲线、诊断、输入快照、native-layout states/actions和运行manifest；加入六场景CTest回归。

## 本轮实际结果

最终Release二进制：`../../tmp/build_flask_runtime/flask_plan.exe`。Windows、双精度、标量DF恢复、AVX2 double4×2球碰撞、额外连续球几何检查；不是作者完整float8实现。

运行命令（从本项目目录）：

```powershell
& 'D:/NEWFILE/Python313/python.exe' tools/benchmark.py --executable ../../tmp/build_flask_runtime/flask_plan.exe --output results/rrtconnect_six_final_20261012 --seeds 41 42 43
```

输出目录必须不存在；复跑时更换目录。每场内部10秒、进程15秒上限，未触发加时或备用算法。

| Scenario | Success | Traj. Len. (m), mean | Plan. Time (ms), mean | Time SD (ms) |
| --- | --- | --- | --- | --- |
| dynobench_uni_bugtrap | 3/3 | 12.658 | 48.9726 | 30.0515 |
| dynobench_uni_wall | 3/3 | 3.62103 | 18.0778 | 7.11183 |
| dynobench_quad2d_bugtrap | 3/3 | 11.7752 | 12.3917 | 10.1161 |
| dynobench_quad2d_hole | 3/3 | 8.97999 | 6.47833 | 0.290371 |
| dynobench_quad3d_block | 3/3 | 8.86542 | 16.6739 | 10.052 |
| dynobench_quad3d_window | 3/3 | 7.00844 | 51.8155 | 33.0528 |


**18/18真实成功，三seed均固定事先列出；不是挑成功seed。** CTest **19/19** 通过，其中6项就是这六配置的seed41运行，日志见 [rrtconnect_ctest.log](rrtconnect_ctest.log)。三seed均值/样本SD只代表本机有限运行；不能据此推断渐近最优性或实验概率常数。

原始记录：[runs.csv](../results/rrtconnect_six_final_20261012/runs.csv)、[manifest.json](../results/rrtconnect_six_final_20261012/manifest.json)、[六场景轨迹图](../results/rrtconnect_six_final_20261012/paths.png)。图只展示原始box和轨迹中心线，不是碰撞检查结果。

## 与截图数值比较

| 场景 | 图中长度(m) | 本地3seed均值(m) | 图中规划(ms) | 本地3seed均值(ms) |
| --- | --- | --- | --- | --- |
| Uni Bugtrap | 12.39 | 12.6580 | 6.0 | 48.9726 |
| Uni Wall | 4.82 | 3.6210 | 0.6 | 18.0778 |
| Quad2D Bugtrap | 11.46 | 11.7752 | 2.7 | 12.3917 |
| Quad2D Hole | 5.16 | 8.9800 | 0.4 | 6.4783 |
| Quad3D Block | 8.08 | 8.8654 | 0.9 | 16.6739 |
| Quad3D Window | 5.49 | 7.0084 | 0.5 | 51.8155 |

图中值来自用户附件；本地值来自CSV，不宣称相同参数/硬件的公平速度对比。Unicycle Wall更短也不能宣称算法更优：单球机器人、box球覆盖、NN/range、采样/lift、R等本地选择与私有实验配置不完全可核实。Hole等场景还有明显绕行，当前shortcut优化的是Eq.20 cost，不能等同最短米制长度。

## 契约与尚未完成的部分

- 当前是DynoBench**数据场景**运行，未链接官方DynoBench/FCL runtime、未调用官方checker。native-layout输出只是可视化/后续验证入口，末步可能不足dt，控制不是零阶保持的官方step结果。
- r2 quad只强制y/dy边界。seed41四个quad的原状态端点误差约0.014–0.111、最大拼接误差约0.068–0.114（直接状态向量范数，混合位置/姿态/角速度单位，只作诊断）。Unicycle对应误差约1e-15。不能将quad降阶成功当作完整原状态连续动力学认证。
- 仍未复现作者float8 BVP/DF全流水线、原VAMP实现参数和实验硬件，也未匹配截图毫秒性能。公开OMPL port用于核对机制，不替代私有实验源码。
- 连续球几何检查仅几何；omega/电机/力矩/其他奇异性仍有限采样，不能授予完整轨迹证书。标准FLASK-RRTConnect无自动渐近最优性。

## 保留的诊断轮次

`rrtconnect_r2_review_20261012`（5/6）、`rrtconnect_target_policy_20261012`（4/6）、`rrtconnect_pose_sampler_20261012`（4/6）、`rrtconnect_incoming_review_20261012`（5/6）和`rrtconnect_range_review_20261012`参数诊断全部保留。它们属于不同代码/配置的功能诊断，部分与重编译重叠，不能混成最终性能统计。最终统计仅上方固定版本18轮；未用成功输出覆盖早期失败。

公式和每条假设的代码/测试/范围/例外仍见 [Theory-to-Code Matrix](theoretical_guarantees.md)。
