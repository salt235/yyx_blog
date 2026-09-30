---
title: "PX4 调参 Part 3：Rate PID 调参"
createTime: 2026/09/16 00:00:00
permalink: /notes/UAV/px4/tuning/part-3-rate-pid.html
---

# PX4 调参 Part 3：Rate PID 调参
> 来源：[PX4 Ultimate Tuning Guide Part 3 The Rate PID Controller](https://www.youtube.com/watch?v=8nWW17ZDOYQ&t=152s)（Chris Rosser）
> 前置：[PX4 调参 Part 2 陀螺仪滤波流程总结](/notes/UAV/px4/tuning/part-2-gyro-filtering.html)（滤波器没调好不要开始 PID）
> 相关：[PX4 滤波与 PID 调参](/notes/UAV/px4/pid-tuning.html)、[PX4 多旋翼四个控制环](/notes/UAV/px4/control-loops.html)
## 1. 控制结构与被调对象
- 控制栈自下而上：**滤波器 → Acro(Rate PID) → Stabilize → Altitude → Position → 自主导航**，必须自下而上调。
- Acro 模式含 3 个独立的角速度环 PID：Roll、Pitch、Yaw，输入是目标角速度与陀螺实测角速度之差（误差）。
- 各项作用：
  - **P**：像弹簧，误差越大回推越强；
  - **D**：像减震器，抑制角速度的变化，防止 P 引起振荡；
  - **I**：累积误差，消除长期偏差（响应慢）；
  - **FF**：与目标角速度成正比，多旋翼只在 **Yaw** 上有用，Roll/Pitch 不用（会造成超调）。
- **调参顺序**：先 D → 再 P（定 PD 平衡）→ 再一起放大 P、D（即 K 值）→ 再调 I（定 PI 平衡）→ 最后 FF。P 把误差压小，同时防止 I 积分饱和振荡。
## 2. 试飞前设置与手法
- `MC_AIRMODE` 设为 **Disabled**：过调导致振荡时不会越振越爬升（避免飞走、摔倒）。
- 日志沿用 Part 2 的设置即可；想省空间可关掉 raw FIFO high rate IMU gyro。
- 模式：**Stabilized**（自己当油门控制器）；若用 Altitude 模式则油门环必须已大致调好。
- 手法：对三个轴分别做**干脆利落的左右/前后/偏航急摆**，幅度可小可大；**绝不能松杆让杆回弹**（会产生振荡输入）；每个轴 10–20 秒即可，然后降落。
## 3. 分析日志：先定 PD 平衡
1. 下载日志 → 上传 Flight Review → 看 **Angular Rate** 图，用 box zoom 放大到某个轴的机动段。
2. 比较 **rate estimated（实测）** 与 **rate set point（目标）** 的峰值高度：
| 现象 | 结论 | 操作 |
| --- | --- | --- |
| 两个峰值高度基本一致 | 临界阻尼，PD 平衡良好 | 保持，进入 K 值调参 |
| estimated 峰值 > setpoint（如 160 vs 140） | 欠阻尼，D 不足 | **增大 D** |
| estimated 峰值 < setpoint（如 150 vs 190） | 过阻尼，D 过多 | **减小 D** |
3. 修改位置：QGC → PID Tuning → Rate Controller → 选 **manual tuning** → 用 **Differential gain** 滑块，每次改 **10%–20%**，再飞再验，可能需要多轮。
4. Roll 与 Pitch 相互独立，可一次飞行同时做两个轴的机动、同时分别改 D。
5. **避免目标角速度饱和**：PX4 默认最大角速度 220 °/s，机动峰值打到约 200 °/s 即可。若 setpoint 出现平顶（削顶），该段数据无法用于调参，需把动作做得更缓。
6. **Step Response 图**（Flight Review → Open PID analysis）可作辅助校验：欠阻尼 = 超调后回弹振荡；过阻尼 = 上升很慢、迟迟到不了 1.0。但并非所有控制器都有阶跃图，最终要练会用时间序列图调参。
## 4. 再调 K 值（整体 PID 倍数）
- K 同时缩放 P、I、D：**PD 平衡正确后，增大 K 不改变平衡，只减小实测与目标的延迟、提高响应**。PX4 没有 D 前馈，K 是唯一能降低延迟的手段。
- 方法：逐步增大 K（QGC 的 overall multiplier），直到出现振荡/嗡鸣（trilling），记下这个上限，再按用途回退：
  - 追求极致机动：回退 **20%–30%**；
  - 稳态悬停/自主飞行、安全优先：可回退约 **50%**。
- KD 上限：PID 调参页最大 3，参数表可到 5（超过 5 会警告），小机型常需要大于 3。
- 术语：K 太低 = **undertuned**（软绵绵、不稳、响应慢）；K 太高 = **overtuned**（悬停或急动作时嗡鸣、自激振荡）。
- **配置选择**：调 D（PD 平衡）用**最重配置**（最大载荷/转动惯量），这样减载后只会更偏过阻尼；调 K 用**最轻配置**（最易振荡），回退后加载也不会振。
- Yaw 轴没有 D 可调，直接调 K：同样增到微振荡再回退 20%–30%；因 K 增大即 P 增大而 Yaw 自然阻尼不变，若出现实测超调，可给 Yaw 加一点小 D 后继续加 K。
## 5. I 项
- I 的可调范围很宽，且 K 已按比例放大 I，通常**不需要怎么调 I**。
- I 增益不改变 I 的强度，只加快积分建立速度：太快会先冲过头再反向积分，产生**比 P 振荡更慢的振荡** → 调小即可。
- 需要极致机动（如翻转后要快速反向）时可适当加大 I。
- **Yaw 的 I** 特殊：若 K 已加到上限、D 为 0，estimated 仍追不上 setpoint（表现过阻尼），可显著加大 Yaw I 来帮助跟踪（可能略超调，再小幅回调）。
- 参数位置：I gain 滑块，或参数表中的 `MC_ROLLRATE_I` / `MC_PITCHRATE_I` / `MC_YAWRATE_I`。
## 6. Yaw 前馈（FF）
- 适用场景：Yaw 在大 K 下仍存在稳定的“跟不上”（undershoot）。FF 按目标角速度大小额外给一点电机差动。
- `MC_YAWRATE_FF` 从约 **0.005** 起调，每次按 **×1.2 / ÷1.2（约 20%）** 调整；FF 主要补幅度不足，几乎不改善延迟。
- 最终往往需要“少量 FF + 少量 I”配合，使 Yaw 跟踪最佳；Roll/Pitch 不加 FF。
## 7. 完成与后续
- 完成标志：Roll/Pitch 的 PD 平衡正确（峰值等高）、K 尽量大且无振荡、Yaw 的 K/I/FF 跟踪良好。
- 下一步：Angle 控制器 → Altitude / Velocity / Position 控制器（Part 4）。
