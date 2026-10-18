# FLASK-RRTConnect首解实测（2026-10-12）

关闭shortcut；seed41/42/43；每场3/3成功。首解时间使用search_seconds，包含成功路径验证，不含配置加载、文件输出和长度积分。cost和长度都对应未经后处理的第一条解。

| 场景 | 首解时间(ms)，均值 ± SD | 首解cost，均值 ± SD | 首解长度(m)，均值 ± SD |
| --- | --- | --- | --- |
| uni_bugtrap | 74.475 ± 64.455 | 52.952 ± 7.843 | 13.997 ± 1.258 |
| uni_wall | 27.064 ± 10.526 | 23.778 ± 17.002 | 6.720 ± 5.323 |
| quad2d_bugtrap | 19.828 ± 19.599 | 31.329 ± 16.479 | 15.493 ± 6.318 |
| quad2d_hole | 8.899 ± 3.715 | 30.707 ± 4.067 | 14.339 ± 2.233 |
| quad3d_block | 19.625 ± 15.849 | 18.068 ± 6.379 | 11.304 ± 2.968 |
| quad3d_window | 70.186 ± 45.554 | 50.131 ± 19.910 | 19.010 ± 6.644 |

cost = integral(w^T R w) + rho*T；Unicycle R=10I，quad R=I，rho=1。cost不是米制长度，不宜跨机器人直接比较。

这是独立规划器在DynoBench数据场景上的实测；quad为reduced-r2契约，未获得官方动力学checker认证。不是论文表格数值。

原始数据：[runs.csv](runs.csv)，配置和源码指纹：[manifest.json](manifest.json)。
