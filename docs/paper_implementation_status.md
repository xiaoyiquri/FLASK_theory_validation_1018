# 对照截图第一列：FLASK-RRTConnect

范围为 DF + RRTConnect 与六个 DynoBench 场景。依据 [论文 Section V / VII-A](https://arxiv.org/html/2603.16059v4) 和 [作者公开 OMPL port](https://github.com/ompl/ompl/pull/1472)。其他表格列及机械臂实验不再作为此任务的缺失功能。

| 环节 | 代码与审查结果 | 范围 / 已知差异 |
| --- | --- | --- |
| 平坦链 | vendor积分链内核，六场景r2，Quad2D n2、Quad3D n4 | 高阶模板仅保留作审计/显式实验 |
| Eq.20–28 BVP | bvp.h：完整flat边界，SPD R，rho=1，枚举正时间驻点取最低成本 | 数值根隔离；未复现作者SIMD闭式四次根 |
| 双向树 | planner.h：nearest、extent metric、range/BVP插值、connect | start取outgoing前段、goal取incoming末段；本地kinodynamic适配，区别于公开几何port双向同插值；range显式本地配置 |
| Algorithm 3 | model.h/validation.h：样本物理映射/限幅、八点球碰撞；Unicycle r2速度极值补充检查 | 极值检查覆盖非零速度条件，其他约束仍有限采样；DF标量、碰撞AVX2 double4×2；不是完整有效性证书 |
| 原控制限制 | io.cpp：Unicycle v/omega、Quad2D电机、Quad3D推力/力矩 | heuristic max_acc不额外变成物理约束；速度/转速用norm；输入模型快照输出 |
| Practical quad r2 | 修正六配置；不再默认full-r4采样/高阶边界 | y/dy连续；姿态/omega端点与拼接误差诊断，不授予完整动力学保证 |
| Algorithm 4 | 跨节点BVP shortcut，重新检查并严格降低cost | 不创造渐近最优性；搜索/后处理时间分开 |
| 六场景运行 | benchmark.py默认六配置，固定seed、预算、日志 | 失败如实保留，不自动搜种子或加时 |
| 表中指标 | Plan::length积分平移速度；CLI/CSV的m、ms | cost另列，计时不含长度积分/I/O；本机时延不等于论文性能 |
| DynoBench | environment/model导入，精确曲线和viewer states/actions导出 | 没有官方runtime/FCL认证；零阶保持、末步和reduced拼接需要另验 |
| Section VI | 原文档、三个用户指定理论测试保留 | r2 quad不满足全部Assumption 1；RRTConnect无AO保证 |

此前偏离目标的SST/DP-RRT入口已从CLI移除。原审计测试/历史记录保留，不把其功能列作当前目标。旧 `examples/time_budget` 与 `results/time_budget_20261012` 使用full-r4，其4/6结果不能代表本轮r2版本。

此前只输出cost，缺少Traj. Len.；现在单独计算米制长度。此前关闭拼接检查后仍强制quad高阶物理端点，当前两个策略分别显式控制，默认严格，仅r2 quad允许降阶端点契约。该选择既能运行实际reduced规划，也暴露其物理误差。

原论文允许quad降到r2用于实际规划，并不消除降阶边界问题。原VAMP实验参数、完整float8 BVP/DF流水线未复现，因此不能宣称匹配截图0.4–6ms或官方DynoBench动力学认证。
