# FLASK-RRTConnect：seed20 / R=I / rho=1 / shortcut开启

日期：2026-10-12。单次固定seed20，无均值统计。其余采样、geometry、验证策略和停止预算采用原六配置；内部时间上限10秒、进程限制15秒。早期运行记录保留。

| 环境 | 结果 | 首解时间(ms) | 总规划时间(ms，含shortcut) | 最终cost | 最终长度(m) | 接受shortcut次数 |
| --- | --- | --- | --- | --- | --- | --- |
| uni_bugtrap | timeout | — | 10000.014 | — | — | 0 |
| uni_wall | 成功 | 1733.396 | 1745.980 | 48.117 | 25.516 | 1 |
| quad2d_bugtrap | 成功 | 19.371 | 29.023 | 22.809 | 14.099 | 3 |
| quad2d_hole | 成功 | 5.080 | 6.869 | 11.040 | 5.647 | 2 |
| quad3d_block | 成功 | 8.456 | 13.427 | 12.711 | 9.278 | 1 |
| quad3d_window | 成功 | 27.392 | 30.829 | 13.031 | 6.487 | 1 |

5/6成功；Unicycle Bugtrap在10秒上限内无首解，该行总时间是失败搜索耗时，不是首解时间。首解时间包含成功路径验证，最终cost和长度是shortcut后的同一条输出路径。总规划时间包含shortcut，排除配置加载、长度积分与I/O。

cost = integral(w^T R w) + rho*T；六场景R=I、rho=1。shortcut仅接受cost下降的替换，不等于最短米制长度。开启shortcut不保证每场都能找到可接受替换。

当前使用DynoBench数据场景、单球/球覆盖几何；quad为reduced-r2契约，未经官方动力学checker认证。

[原始运行CSV](runs.csv) · [完整指标CSV](metrics.csv) · [二进制与配置指纹](manifest.json)
