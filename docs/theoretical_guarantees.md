# FLASK Section VI：理论一致性与 Theory-to-Code Matrix

核对日期：2026-10-11，实现更新：2026-10-18。论文版本固定为 [Duong 等，Ultrafast Sampling-based Kinodynamic Planning via Differential Flatness，arXiv:2603.16059v4](https://arxiv.org/html/2603.16059v4)，重点为 Section VI、Eq.31–38、Definitions 2–5 和 Algorithm 3。
本文公式按该版本编号；下述代数推导和反例是本项目独立分析。

初次审计只读引用 `../../RRTstarDFPFF_paper_r2_float32_strict_20261010`，以下称“参考工程”。它实现的是 RRT*/PFF 路线，不能改称作者的 FLASK-RRTConnect。
2026-10-12 已将必要 header 固定为 `vendor/rrtpff` 的带哈希本地快照，并新增独立 BVP/RRTConnect/验证/后处理。按用户最新范围，传播/SST/DP代码已移除，六场景采用practical r2；理论审计保留，不额外实现其他规划器。当前集成结果见 [rrtconnect_six_review.md](rrtconnect_six_review.md)，历史full-r4结果见 [runtime_validation.md](runtime_validation.md)。
作者项目页的代码发布说明不能替代实际源码核查：[FLASK 项目页](https://kavrakilab.org/flask/)。

## 1. 变量、代价和适用前提

| 符号 | 定义与注意事项 |
| --- | --- |
| `n` | 平坦输出 `y` 的维度，不是原始机器人状态维度 |
| `r` | 积分链阶数；`w=y^(r)`，`z=(y,dy,...,y^(r-1))`，维度 `rn` |
| `l,m` | 状态映射 `alpha`、控制映射 `beta` 所需的最高输出导数阶数 |
| `p` | 对原始状态的第 `p` 阶导数施加约束时的阶数 |
| `R,rho` | `R` 是固定 SPD 权重，`rho>0` 为时间惩罚；理论代价 `C=integral(w^T R w dt)+rho*T` |
| `G_T` | 带 `R^-1` 的可控 Gramian，积分 `exp(At) B R^-1 B^T exp(A^Tt)` |
| `d_T` | `z_f-exp(AT)z_0`；固定时间最小能量为 `d_T^T G_T^-1 d_T` |
| `T*` | 最小化上述能量加时间代价的正时间；不是单纯最短运动时间 |
| `pi,sigma` | 参考可行轨迹和分段 BVP 轨迹；可具有不同总时长 |
| `N,M` | 样本/节点数和 trace 的段数；不是碰撞检查采样数。`M` 通常随 `N` 增大 |
| `epsilon,delta` | 相对代价误差和到整个无效状态集的正 clearance |
| `zeta_N,eta_N` | 每段的代价上界与到参考轨迹的距离尺度，单位不同 |
| `D` | `(rn+r²n)/2 = nr(r+1)/2`，不是简单用 `n` 或 `rn` 替代的欧氏维度 |
| `mu(Z_free)` | 选定平坦空间坐标下自由域的 Lebesgue 体积，要求有限且正 |
| `kappa,C_mu,C_p,L_alpha` | 收敛界参数、覆盖体积常数、平坦轨迹距离常数和逆映射 Lipschitz 常数 |

引用 Theorem 1 的完整前提应包括：正整数 `n,r` 的可控积分链、固定 SPD `R`、正时间惩罚、最优时间的完整端点 BVP、Assumption 1、可表示为平坦状态无效集的所有约束、理论上无约束的 `w`、正 clearance 的参考轨迹、足够的采样覆盖以及完整轨迹有效性判定。将结论移到原状态空间还要检查 `alpha` 的 Lipschitz 性和所用角度/姿态坐标。

采样覆盖不能从树节点计数直接推出。作为可核查的充分条件，应使用有界自由域上独立均匀样本；其他采样分布需要正密度下界等额外论证并相应修改常数。仅采样输出、依靠 PFF 自动生成导数，不等价于在 `R^(rn)` 独立均匀采样。相关覆盖论证来自论文引用的 [Schmerling、Janson、Pavone，drift case](https://arxiv.org/abs/1405.7421)，不能只以某个 seed 的成功率替代。

## 2. Assumption 1 与 Definitions 2–5

**Assumption 1** 要求 `r>l`，从而 `x=alpha(z)`；状态不能再依赖当前 pseudo-control 或局部边的隐藏高阶导数。这只是状态恢复条件，不足以证明所有控制约束也成为状态约束，更不足以自动给出全局 Lipschitz 常数。

**Definition 2** 的 clearance 是整个轨迹在相应状态空间中到无效集的最小距离：

\[
\inf_{t\in[0,T_\pi]}\operatorname{dist}(\pi(t),X_{obs})\ge\delta>0.
\]

定理 (i) 需要的是平坦状态空间的 clearance；不能用工作空间位置的障碍距离直接替代，尤其在加入速度、转速和电机约束后。边的离散采样全部通过不等于正 clearance。

**Definition 3 / Eq.31** 要同时满足三项：

\[
C(\sigma)\le(1+\varepsilon)C(\pi),\qquad
C(\sigma_i)\le\zeta\quad\forall i,
\]
\[
\sup_{t_\sigma\in[0,T_\sigma]}
\inf_{t_\pi\in[0,T_\pi]}
\|\sigma(t_\sigma)-\pi(t_\pi)\|\le\eta. \tag{31}
\]

最后一项是从 `sigma` 到 `pi` 的单向轨迹集合距离，不是同时间跟踪误差，也不是对称 Hausdorff 距离。测试以 `pi(t)=t`、`sigma(t)=t²` 验证其区别；这项测试不代替前两项代价条件的证明。

**Definition 4 / Eq.32**：控制或高阶状态导数约束可写成

\[
\gamma(y,\dot y,\ldots,y^{(q)})\le0,
\qquad q=\max\{m,l+p\}. \tag{32}
\]

如果不存在状态导数约束，就只需相应的 `m`，不能人为引入 `p`。为了让所有这些约束只依赖 `z`，须选择 `r>=q+1`。理论推导保持 `w` 无约束，通过更高阶平坦状态中的坐标表示原始控制限幅。

**原文集合表达需修正其语义**：`gamma<=0` 是可行条件，因此无效域应是

\[
Z_{obs}=\alpha^{-1}(X_{obs})\ \cup\
\{z:\text{至少一个 }\gamma_j(z)>0\}\ \cup\
\{z:\alpha,\beta\text{ 在所用分支上未定义}\}.
\]

原文将碰撞条件和 `gamma<=0` 同写在一个集合中，按字面不能代表所有无效状态；复现时应按照“碰撞或违反任一约束即拒绝”的逻辑实现。为正 clearance 分析，边界的归属和奇异集也须明确定义。

**Definition 5** 假定在同一问题、同一 Eq.20 代价下存在达到最小值 `C*` 的可行轨迹。不能把一个求解器的最小候选值视为全局 `C*`，也不能把平坦加速度/jerk/snap 代价与实际电机能耗、轨迹长度或最短时间直接等同。优化后者需另建目标和证明。

## 3. Theorem 1 与 Eq.33–35

按 v4 的 Eq.33 记号：

\[
C_{Z_{free}}=C_\mu^{-1}D^{-1}
6^{rn+r^2n/2}2^{rn/2}\mu(Z_{free}),
\]
\[
\zeta_N=(1+\kappa)^{1/D}C_{Z_{free}}
\left(\frac{\log N}{N}\right)^{1/D},\qquad
\eta_N=C_p\zeta_N. \tag{33}
\]

这里保留原文 `C_Zfree` 在根号幂外的写法，不暗中用其他论文的常数归一化替换。可稳妥引用的是固定常数下的阶数
`zeta_N,eta_N=O((log N/N)^(1/D))`。测试令 `C_Zfree=1,C_p=2,kappa=1` 作为符号数值夹具，验证 `N>e` 时尺度递减；这些数值没有实验概率含义。

设 `A_N` 为平坦空间中存在满足三项 trace 条件的事件，则

\[
1-P(A_N)=O\left(N^{-\kappa/D}(\log N)^{-1/D}\right). \tag{34}
\]

`kappa=0` 时该界仍随 `log N` 缓慢趋零。大 O 隐含阈值和乘法常数，不给定某个有限 `N` 的具体成功概率；不能写成精确等式或擅自给出“99.9%”。这类存在性结论也不等于任意搜索器必定选择并找到该 trace。

若所用区域内 `alpha` 满足
`||alpha(z1)-alpha(z2)||<=L_alpha||z1-z2||`，则 `A_N` 蕴含原始空间中距离上界 `L_alpha*eta_N` 的事件 `B_N`，并有

\[
1-P(B_N)\le1-P(A_N)
=O\left(N^{-\kappa/D}(\log N)^{-1/D}\right). \tag{35}
\]

代价一致性是在两种表示中使用同一平坦代价的意义下成立。`L_alpha` 不只是“函数连续”；连续映射不能自动保证线性距离上界。

实际 DynoBench 环境可能只有 YAML 边界、碰撞后端和离散约束检查，不能精确得到平坦自由域体积、全局逆映射导数界、覆盖常数和轨迹扰动常数。即使本项目能解析算出积分链的 Gramian 常数，也不能由此准确标定环境中的 `C_mu,C_p,L_alpha` 或有限样本失败概率。本项目不将它们设为实测常数，不增加理论半径作为默认搜索参数。

## 4. Eq.36–37：任意积分链的可控性与指数

使用 derivative-major 排序，`A` 的块超对角线为 `I_n`，`B` 的最后一块为 `I_n`。第 `j` 个输入的列满足

\[
A^k b_j=e_{(r-1-k)n+j},\quad 0\le k<r;
\qquad A^k b_j=0,\quad k\ge r.
\]

因此 `[B,AB,...,A^(r-1)B]` 是分块反向排列的单位基，包含全部 `rn` 个独立方向。标准可控性矩阵至 `A^(rn-1)B` 已足够；Eq.36 印到 `A^(rn)B` 多出的列全为零，不改变秩：

\[
\operatorname{rank}C=rn. \tag{36}
\]

截断矩阵至 `A^(k-1)B` 的秩为 `n*min(k,r)`，秩增量在前 `r` 步均为 `n`、以后为零，所以各输入的可控性指数为

\[
\nu_1=\cdots=\nu_n=r. \tag{37}
\]

这个论证对所有正整数 `n,r` 成立，与机器人逆映射是否合法无关。`test_controllability.cpp` 实际构造矩阵、检查每个截断秩、每个输入链的独立性和零尾，并用删去一个输入的反例确认测试能检测退化。
默认扫描 48 组运行时维度和 9 组参考模板实例；`n=5,r=7` 为额外 CTest。用户可传入其他资源范围内的 `n,r`。有限数值测试不冒充所有正整数的穷举证明；更大/更高阶情况用上述符号证明，必要时换稀疏矩阵或高精度实现。

## 5. Gramian determinant 与 Eq.38：独立推导

令 `Q=R^-1`、`p_i=r-1-i`，则积分链 Gramian 的第 `(i,j)` 块为

\[
(G_T)_{ij}=Q\frac{T^{p_i+p_j+1}}{p_i!p_j!(p_i+p_j+1)}.
\]

记 `H_ij=1/(p_i!p_j!(p_i+p_j+1))`，`S_T=diag(T^(p_i+1/2)I_n)`，于是

\[
G_T=S_T(H\otimes Q)S_T,
\]
\[
\det G_T=(\det H)^n(\det Q)^r T^{nr^2}. \tag{*}
\]

`H` 是单项式基的积分 Gram 矩阵，正定；`Q` 固定且 SPD，故常数 `C=(det H)^n(det Q)^r>0`。对理想齐次积分链，(*) 对每个 `T>0` 都精确成立，比仅小时间 `Theta(T^(r²n))` 更强。`R` 随时间/维度缩放、不同轴采用不同阶数或系统不再为该积分链时，不得沿用此精确式。

例如 `r=1` 时 `det G_T=det(Q)T^n`；`r=2,R=I` 时每轴块行列式为 `T^4/12`，总值为 `12^-n T^(4n)`；`r=4` 的指数是 `16n`，不是 `4n`。这里求的是完整 `rn` 维 Gramian；PFF 约束投影后的较小矩阵通常有不同幂次，不能替换。

`test_gramian_scaling.cpp` 用固定带交叉通道项的 SPD `Q`：

- 在 `T=10^-1,3*10^-2,10^-2,3*10^-3,10^-3,10^-4` 检查正定性、归一化行列式常数与 log/log 斜率。
- 用 Gauss–Legendre 数值积分逐块交叉校验解析式和现有模板内核，避免只用同一公式自我验证。
- 低阶情况直接计算 determinant；高阶计算 `log(det G_T)`，剥离时间缩放后 Cholesky 分解，避免下溢和条件数误判。
- 拒绝零/负时间与非 SPD 权重；高阶 Hilbert 矩阵仍可能数值病态，不能把浮点分解失败解释为系统不可控。

Gramian 椭球的体积是

\[
\mu(E_\psi)=\mu(B_{rn})\psi^{rn}\sqrt{\det G_T}
=\mu(B_{rn})\psi^{rn}\sqrt C\,T^{nr^2/2}. \tag{38}
\]

若只知道 `det G_T>=C1*T^(nr²)`，体积下界系数应是 `sqrt(C1)*mu(B_rn)`。原 Eq.38 显示 `C1`，后文又定义 `C_mu=sqrt(C1)*mu(B_rn)`；应区分行列式常数和体积常数，或明确重新命名常数，不能机械混用。

## 6. 理论邻域与实际 NN/extension 的区分

理论 cost-based neighborhood 可以定义为有向集合

\[
\mathcal N^+_N(z)=\{z':c^*(z,z')\le\zeta_N\},
\quad c^*(z,z')=\inf_{T>0}[d_T^TG_T^{-1}d_T+\rho T].
\]

漂移系统通常不能假定 `c*(z,z')=c*(z',z)`。这既不是欧氏球，也不是输出空间的固定距离；邻域入边与出边需区别。代价阈值用于理论连接/重连覆盖，不直接规定树扩展步长。

标准 RRTConnect 的 nearest-neighbor 选择由所配置距离函数决定，extension/connect 由所配置步长、方向、生长方式和局部有效性检查决定。换成 FLASK BVP 不会自动创建 Eq.33 邻域，也不会自动加入重连。当前已找到作者公开的 [OMPL 移植 PR #1472](https://github.com/ompl/ompl/pull/1472)，固定核对 commit `fe44ef0448e098988772df074507e9f143570e83`；PR 中同时说明原论文 VAMP 实验实现仍是私有代码，不能将两者混为同一实验版本。

公开移植版 `FlatStateSpace::distance` 默认使用各输出/导数子空间距离按extent加权的compound metric，另可显式选择trajectory cost；BVP插值在`fraction*T*`处取状态，growTree用`maxDistance/distance`为fraction，两棵树都从nearest→sample插值，然后按各自方向验证。range没有Eq.33常数公式。来源见 [参考源码](../references/ompl_fe44ef0/SOURCE.md)。本项目采用同一extent NN和start BVP插值，当前goal树取incoming BVP末段是本地kinodynamic适配，不能冒称公开port或私有VAMP实验默认策略；range、拓扑/NN及rho仍需按实际版本核对。

参考工程中能确定的实际机制是：

- `core/nearest_neighbor.h` 的 `nearest/near/nearestK` 调用 `distanceToOutput`，候选检索在输出空间进行。
- `core/rrtstar.h` 先 nearest 再 `limitExtension(...,config.range)`，实际重连半径为 `rewire_factor*range*(log(N)/N)^(1/Model::FlatDim)`。
- `systems/dyno_model.h` 的普通 PFF 扩展可以释放终端导数，重连用固定完整平坦端点；有限时间候选、边拒绝和模型约束也参与选边。

这是参考工程的 RRT*/PFF 行为，幂次和空间均与 Eq.33 不同。不能据函数名 RRT*、存在重连或 BVP 代价排序，就断言它已满足本文随机图论证。该工程的行为也不能归为作者的 FLASK-RRTConnect 默认配置。

## 7. Unicycle：非零速度、零速度与控制约束

对原状态 `(x,y,theta)`，固定 forward/reverse 分支后，`l=1`：位置由 `y` 恢复，朝向由非零 `dy` 的方向恢复。r=2 因而满足 `r>l`，状态与 `w=ddy` 无关。参考工程 `df/unicycle_df.h` 在速度小于等于 epsilon 时拒绝逆映射，并将速度分支保存在 metadata。

在 `||dy||>=v_min>0` 的有界域、固定分支及连续角度图内，`atan2` 的导数大小有 `1/v_min` 上界；加入位置分量后可给出有限的局部 Lipschitz 界。若使用圆周角距离，应在证明中明确换用对应 metric。若坚持把 wrapped `theta` 看作全局实数坐标，则 `+pi/-pi` 接缝使映射发生坐标跳跃；仅排除零速并不能保证所有坐标上的全局 Lipschitz。

零速度问题不是“设置 epsilon 后理论上解决”。取 `dy=(epsilon,0)` 和 `(0,epsilon)`，两者都趋于同一个零向量，朝向却分别趋于 `0` 和 `pi/2`，因此不能在零速度点连续扩展位置输出的单值逆映射。停驻时朝向不能由该有限阶 flat state 唯一决定。forward/reverse 切换也不能越过零速而继续沿用固定分支的论证。

控制为 `v=+/-||dy||`、`omega=(dx*ddy-dy*ddx)/||dy||²`。速度约束可由 r=2 的 `z` 表示；转速约束依赖 `ddy=w`，所以完整 Definition 4 的状态化条件需要 `r>=3`。r=2 逐样本检查 omega 虽有实际意义，却不等于理论所用“无约束 w、所有限制仅在 Z_obs”的模型。

测试验证两个分支的非零 roundtrip、状态与 w 无关、控制随 w 改变、转速违规、固定角度图的局部界、零速拒绝、趋零导数发散及角度接缝。有限样本的 Lipschitz 检查不代替解析界的证明。

## 8. Quad3D reduced-r2 与 full_df

平坦输出为 `(p_x,p_y,p_z,psi)`。完整姿态需要位置加速度和 yaw，完整角速度还需要 jerk 和 yaw rate，因此完整状态映射有 `l=3`；控制需要 snap 和 yaw acceleration，统一最高阶 `m=4`。

| 模式 | 可以直接支持的结论 | 不能直接支持的结论 |
| --- | --- | --- |
| reduced-r2 (`paper_r2`) | 8 维 `(y,dy)` 积分链的 rank、指数与 Gramian；单条 cubic 内高阶导数可计算 | `r>l` 不成立；不存在仅由 reduced 节点确定的完整姿态/角速度映射；原电机控制限幅不是 reduced 状态集约束 |
| full_df r=4 | 16 维状态含 acceleration/jerk，非奇异域内满足状态恢复的 Assumption 1 | snap 是当前 w，电机/力矩约束仍依赖 w；尚不满足 Definition 4 的全部控制状态化条件 |
| 理论 r>=5 提升 | 统一状态包括至 snap，可把 m=4 原控制约束写成 z 约束；当前已提供 r5 C++/CLI 映射与规划 | 仍需非奇异域、完整轨迹检查、采样及最优性条件，不能由阶数单独推出最优性 |

参考工程 `df/paper_r2_df.h` 仅保存 reduced 边界；`simd/discrete_validator.h` 从 cubic 求至四阶导数，`df/quad3d_df.h` 的完整 frame 重建用加速度/jerk/snap。cubic 的区间内 snap=0 不等于真实轨迹在拼接处 snap 始终为零，也不保证姿态或角速度连续。

同一 reduced 节点的两条边可给出不同 acceleration，因此给出不同姿态；即使 acceleration 相同，jerk 不同也可能给出不同 omega。r=2 拼接仅保证 `y,dy` 连续，无法保证完整状态连续。参考工程 `systems/dyno_model.h` 对 reduced 模式可记录 `reduced_order_result` 和物理边界不匹配，但不会据此强制拒绝；应按其 reduced 结果语义报告，不能称为满足完整 DynoBench 动力学的连续轨迹。

完整映射还排除推力方向向量 `a+g*e3=0`、推力方向与 yaw heading 的叉积为零、不可定义 yaw 的姿态等奇异性。要获得有限 Lipschitz 常数，不能只是“非零”，而要在所用紧域上与这些分母的零集保持正距离，并一致处理 yaw/quaternion 的坐标分支。

测试给出上述非唯一性、状态/控制导数阶数、snap 改变控制、两个 frame 奇异性及 C1 拼接不保证物理连续的反例。测试使用较宽控制边界来隔离映射依赖；不表示 DynoBench 默认电机限幅均可满足。

## 9. 完整有效性、有限采样与 Algorithm 3

理论的有效性要求对全部 `t` 检查碰撞、状态限制、控制限制、奇异性及拼接连续性；正 clearance 同样是连续时间命题。Algorithm 3 只检查有限 `t_j`。SIMD 改变执行批次，不增加被检查的时间集合，也不保证样本之间没有穿障。

反例：点机器人 `x(t)=t,t in [0,1]`，障碍物占 `[0.049,0.051]`。步长 0.1 的所有采样通过，真实轨迹却在 `t=0.05` 相交；步长 0.001 的检查才捕获该例。对有限半径机器人可采用相应的膨胀障碍构造同样的例子。`test_theory_assumptions.cpp` 检查的是这个解析反例，不是完整 DynoBench/FCL 后端测试。

参考工程 `validateDiscrete/SampleGrid` 是有限网格；其他速度区间证书或 Unicycle speed-extrema 检查也不能自动成为所有障碍/控制的连续时间证书。连续碰撞算法、区间包络、Bernstein 保守界或证明机器人移动距离不超过安全余量的自适应细分，可以用于弥补缺口；它们必须实际覆盖全部约束且包括误差，不能只在文档中假设存在。

## 10. 最优性分析的合法引用范围

Section VI-B 的论证需要 Definition 5 中的最优值，以及一列 clearance 轨迹，其代价趋近 `C*`；还需要使 trace 的偏离小于对应 clearance。仅说 `delta_N=O(eta_N)` 不足以保证它们的常数相容，实际证明应保证如 `eta_N<delta_N` 的包含关系，并在原空间使用 `L_alpha*eta_N`。

规划器还必须具有相应的连接/重连或动态规划策略，确实检索和保留这些低代价邻居，并在采样变密时继续优化。只有满足模型、成本、最优时间 BVP、采样覆盖、完整验证等前提的特定 RRT*/FMT* 类型实现，才能援引相应收敛结论。若全局最优 BVP 时间被有限候选或时间上下界替代，还要证明这些限制不会破坏所需的小代价连接。

标准 FLASK-RRTConnect 的首个可行解、shortcut 后处理以及局部边的最优代价都不能自动推出全局渐近最优。不能写“FLASK 将任意采样规划器变成渐近最优规划器”。其可引用的条件结论是：接入适当的渐近最优规划器，并保持相关假设与邻域策略，才可能保留该规划器的保证。

论文此处使用的是**依概率收敛**的界。本项目不把它强化为几乎处处收敛，不用 CTest 通过或有限次数规划成功验证渐近概率定理，也不从参考工程的名称推导出理论保证。

## 11. Theory-to-Code Matrix

下表“参考”路径相对于上述参考工程，其数学/DF header 快照也位于本地 `vendor/rrtpff`；其他生产路径相对于本独立项目。旧工程搜索策略仅作为审计对象，不代表新搜索器。标为未证明/不满足的条件是审计结论，不是测试遗漏的默认豁免。

| 数学假设/命题 | 代码实现或缺口 | 数值测试/证据 | 成立范围 | 已知例外/当前结论 |
| --- | --- | --- | --- | --- |
| Eq.16 / Eq.36，齐次积分链及 rank=rn | 参考 `pff/pff_integrator.h::system` | `test_controllability.cpp` 检查完整及截断矩阵，9 模板与动态构造交叉校验 | 任意正整数 n,r 的符号证明；48 动态维度数值扫描，另 5,7 | 缺失输入负例被捕获；不可外推任意非线性系统 |
| Eq.37，各可控性指数 r | 同上，A 的块移位、B 尾部单位块 | 每条输入链 rank=r；截断秩增量 n | 相同阶数的 n 条积分链 | 混合阶数指数不全相等 |
| 固定 SPD R | 参考 `pff/pff.h` 和 Gramian 接口 | `test_gramian_scaling.cpp` 使用有交叉项的 SPD Q，拒绝奇异 Q | 固定权重、有限正时间 | 非 SPD、时变权重不适用 |
| det(G_T)=Theta(T^(r²n)) | 参考 `IntegratorChain::gramian`；独立精确分解 (*) | logdet 常数、斜率、低阶 determinant、独立 Gauss 积分 | 理想完整积分链；数值时间 1e-4..1e-1 | PFF projected Gramian 不等价；高阶病态/下溢须报告 |
| Eq.38 椭球体积 | 本文解析行列式到体积推导；搜索器没有此量的默认参数 | 依赖已验证 detG，未做蒙特卡洛体积估计 | sqrt(detG) 和正确的常数平方根 | 原式 C1 记号需区别行列式常数 |
| Assumption 1：r>l，x=alpha(z) | 参考 `df/unicycle_df.h`；完整 `df/quad3d_df.h` | `test_theory_assumptions.cpp` 改变 w 后比较状态 | Unicycle r2 固定非零分支；Quad3D full r4 非奇异区域 | Quad3D reduced-r2 不满足 |
| Eq.35 的 Lipschitz alpha | 参考零速/frame 奇异拒绝；没有全局 L_alpha 证书 | 局部非零速度界、零速方向极限、atan2 接缝、frame 零分母 | 紧的非奇异域与一致坐标图；局部数值证据 | 零速、分支切换、yaw/姿态接缝、趋近零推力不保证全局界 |
| Def.2，正 clearance | 参考碰撞/状态检查，没有所有时刻 clearance oracle | 解析薄障碍反例；本文解释空间区别 | 理论参考轨迹需对完整无效集有正裕度 | 工作空间距离、离散通过均不足 |
| Def.3 / Eq.31 三项 trace 条件 | 参考 `FlatTrajectory` 可求任意时刻；没有全局 trace oracle | t 与 t² 的单向距离示例，只验证第 iii 项语义 | 同一成本、逐段代价和连续轨迹集合距离 | 不代表时间跟踪误差；总代价两项需另证 |
| Def.4 / Eq.32，控制约束只依赖 z，w 无约束 | 参考各 DF 的 `sampleFeasible` 在样本上检查控制 | Unicycle 同 z 不同 w 可违反 omega；Quad3D snap 改变控制 | Unicycle 完整约束状态化需要 r>=3；Quad3D 控制需要 r>=5 | 现有 Unicycle r2、Quad3D full r4/reduced r2 不完整满足 |
| 无效集为违反任一限制的并集 | 参考 `validateDiscrete` 按映射/状态/控制/碰撞失败拒绝 | 具体控制反例；本文集合语义审计 | 包含原约束、奇异点及边界约定 | 不能直接照抄原文碰撞与 gamma<=0 的交集 |
| 最优时间、完整端点 BVP | 新 `include/flask/bvp.h` 固定全部端点导数，Cauchy 根界内枚举正时间驻点并取最低代价 | `test_bvp.cpp` 完整边界、独立成本积分、r2 四次时间条件、多正根全局成本比较、r4/r5 动力学 | 固定 SPD R、rho>0，数值可解析范围；无额外有限候选时间 | 1e-9 时间阈值和数值根精度须考虑；无约束最优边可能不满足物理限制 |
| Eq.33 的 zeta_N/eta_N | 本文公式与符号夹具；无默认半径注入 | 尺度单调递减和 eta/zeta=Cp | 固定正常数、N趋无穷；测试 N=10..1e6 | 环境体积/Cp/Lalpha 未标定；没有有限 N 概率承诺 |
| Eq.34–35 采样覆盖/概率界 | 当前论文profile采用flat box采样，拒绝旧pose-speed采样选项；只拒绝实际新状态/边 | `test_planner.cpp` 固定seed重现；六场景运行不估计概率常数 | 仍需采样覆盖、clearance、成本及完整有效性前提 | 物理端点的固定速度lift不是采样分布；range/拒绝与树节点数也不能直接满足iid论证 |
| 理论 cost 邻域与实际 NN/extension | `planner.h` extent metric、start前段/goal incoming末段BVP插值、正时间connect | `test_planner.cpp` metric、BVP与incoming方向及绕障；旧RRT*只作审计 | NN对照公开port；goal截短为本地kinodynamic适配；没有理论cost-neighborhood图搜索 | Eq.33不是默认半径；私有VAMP参数未知，不能将本地策略冒称作者默认 |
| Quad3D高阶映射与完整状态连续 | `model.h` 从完整jet恢复；`validation.h::path` 两个严格开关默认true，六场景quad r2显式false；`diagnostics.h`与`result_io.h`输出首解/最终/中间路径误差 | `test_models.cpp` 原ODE残差、reduced默认拒绝及显式降阶契约；`test_io.cpp`输出与未经checker认证标记 | 单条非奇异边；r2仅y/dy连续，高阶物理误差需诊断 | 六场景降阶结果不授予Assumption 1或完整动力学认证 |
| Algorithm 3与完整轨迹有效性 | `validation.h::edge` 分散批次检查、可选Bernstein球几何；`model.h::intervalValid` r2 Unicycle速度极值 | `test_validation.cpp`薄障碍/几何/AVX2；`test_models.cpp`网格漏检零速及normalized负例 | r2 Unicycle数值极值检查补非零速度和速度限幅；其他state/control有限采样 | 速度极值/球几何不证明所有约束；根精度与浮点误差仍需考虑 |
| Def.5 / VI-B，全局最优值及鲁棒近似 | 参考工程没有全局 C* 或 clearance 近似序列证书 | 本次不以有限测试证明此无限过程 | 最优存在、clearance 代价趋近 C*、偏离界可容纳 | 仅局部最优 BVP、shortcut、首解不够 |
| 特定规划器的渐近最优性 | 当前RRTConnect不含rewire；结果AO verified=false | `test_io.cpp`不允许输出虚假AO标记；传播/SST已移除 | 只有特定规划器的采样、邻域/重连或DP、有效性及模型前提全部满足才能另证 | 标准FLASK-RRTConnect不自动AO；有限数值PASS不授予证明 |

## 12. 构建和验证记录

独立构建入口和维度选项见 [README](../README.md)。不链接真实 DynoBench/FCL，不需要网络；使用本地 header 快照和新生产代码，实际执行积分链、DF、BVP、搜索与验证。
初次理论构建保留在 `tmp/build_flask_theory`；早期运行构建在 `tmp/build_flask_runtime`。历史理论记录见 [validation.md](validation.md) 和 [runtime_validation.md](runtime_validation.md)。2026-10-18构建为 `build-measurements1018`，首解/预算测量协议与120次运行结果见 [measurement_protocol.md](measurement_protocol.md)、[measurement_results_20261018.md](measurement_results_20261018.md)。有限测试和统计不构成上述定理前提的全局证明。
