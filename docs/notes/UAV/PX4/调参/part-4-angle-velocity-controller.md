---
title: "PX4 调参 Part 4：角度与速度控制器调参"
createTime: 2026/09/16 00:00:00
permalink: /notes/UAV/px4/tuning/part-4-angle-velocity-controller.html
---

# PX4 调参 Part 4：角度与速度控制器调参
> 来源：[PX4 Ultimate Tuning Guide Part 4 Angle Controller, Altitude Controller, Velocity Controller](https://www.youtube.com/watch?v=gNtwzh1c9T4&t=350s)（Chris Rosser）
> 前置：[PX4 调参 Part 3 Rate PID 调参流程总结](/notes/UAV/px4/tuning/part-3-rate-pid.html)（Rate 环必须先调好）
> 相关：[PX4 多旋翼四个控制环](/notes/UAV/px4/control-loops.html)、[PX4 滤波与 PID 调参](/notes/UAV/px4/pid-tuning.html)
## 0. 位置与前提
- 控制栈自下而上：滤波器 → Rate（Acro）→ **Angle（角度）→ 速度/加速度控制器（X/Y/Z）** → 位置与路径。
- 前置条件：Part 1–3 已完成，日志设置与 Flight Review / PID review 已会使用。
## 1. 角度控制器（Angle / Stabilized 模式）
- 结构最简单：**只有 P 项**，没有 I、也没有 D，只需调一个增益。
- 参数位置：QGC → Vehicle Configuration → PID Tuning → **Attitude Controller** → use manual tuning；参数为 `MC_ROLL_P`、`MC_PITCH_P`、`MC_YAW_P`。
- 数据：PID review 里的 roll / pitch / yaw **angle** 图，比较 setpoint 与 estimated 的幅值：
| 现象 | 结论 | 操作 |
| --- | --- | --- |
| setpoint 幅值明显大于 estimated | P 增益太低 | 增大 P |
| estimated 幅值大于 setpoint（上下有明显超调） | P 增益太高 | 减小 P |
| 两者幅值基本一致 | P 增益合适 | 换下一个轴 |
- 三个轴分别调。Roll 与 Pitch 的 P 值通常接近（除非构型很不对称）；**Yaw 的 P 往往要高得多**（Yaw 物理阻尼大），且滞后更大，属正常现象。
## 2. 垂直速度控制器（Altitude 模式）
### 2.1 前置设置
1. `MPC_Z_VEL_MAX_UP` ≈ **3 m/s**，`MPC_Z_VEL_MAX_DN` ≈ **1.5 m/s**：Direct Velocity 模式下满杆直接对应速度设定值，速度上限设太高会猛冲上天或急坠（还可能进入桨叶涡流甚至摔机）。
2. PID Tuning → Velocity Control → **Vertical**，position control mode 设为 **Direct velocity**。
3. 先调 P 增益 `MPC_Z_VEL_P`，再调 D 找 PD 平衡。
### 2.2 试飞动作（Altitude 模式）
- 解锁起飞，等 GPS 高度稳定后再开始。
- 做 **4–5 组**：满油门爬升 → 保持约 1 s 让飞机达到并稳定在最大爬升率 → 立即收到 0 油门下降 → 保持至少 1 s 达到最大下降率 → 再立刻满油门。
- 输入越接近**阶跃**越好，方便看出清晰的阶跃响应；不需要飞很高，但每段必须真正达到最大速度。
### 2.3 看日志调参
| 现象 | 结论 | 操作 |
| --- | --- | --- |
| estimated 上升慢、误差大 | 过阻尼或整体增益偏低 | P 每次 **+20%** |
| 明显尖锐超调 + 回弹（尤其动作末尾） | P 过大 | 减小 P，或增大 D 找 PD 平衡 |
| 快速上升后稳定贴住 setpoint | PD 平衡且增益合适 | 进入整体增益调整 |
- **整体增益**：把 P、I、D 同时按 **×1.2** 逐级放大以提高响应；放大后系统物理阻尼不会跟着变大，可能出现欠阻尼/超调 → 略减 P（或略加 D）把 PD 平衡拉回来，再继续放大。
- **Nyquist 振荡**：速度环只有 50 Hz，P 超过临界值会出现 **25 Hz 及以上**的锯齿状（非正弦）快速振荡，**D 无法抑制** → 把 P 减小约 **30%**；若降到该上限仍欠阻尼，就继续减 D，减到 D=0 仍欠阻尼即到达该控制器的调参极限。
- 完成后飞机在 Altitude 模式下应稳定悬停。
## 3. 水平速度控制器（Position 模式）
### 3.1 前置设置
1. `MPC_VEL_MANUAL` = **2 m/s**：低于 PX4 允许的下限，需要**强制保存**。Direct Velocity 下满杆就是该速度，设大了会满场乱窜、难以调参。
2. PID Tuning → Velocity Control → **Horizontal**，position control mode 设为 **Direct velocity**。
3. 先调 P（及 D）增益 `MPC_XY_VEL_P`、`MPC_XY_VEL_D` 找 PD 平衡。
### 3.2 试飞动作（Position 模式）
- 解锁起飞，等位置稳定（取决于 GPS，几秒）。
- 做 **4–5 次左右扫掠**：向一侧加速到满速 → 反向加速到另一侧满速；再做 **4–5 次前后扫掠**。与角度调参的“原地急摆”不同，这里要真正**加速到满速并保持**。
### 3.3 看日志调参
- velocity 图中速度 setpoint 应接近**方波/阶跃**；若不够方，重飞并用更干脆的满杆，阶跃越清晰越好调 PD。
- PX4 默认的水平速度环常常**过阻尼**（D 偏多、P 偏少），尤其小机型：
| 现象 | 结论 | 操作 |
| --- | --- | --- |
| estimated 长时间追不上 setpoint | 过阻尼 | 先减小 D（很多机型可减到 0 仍不过冲，因为空气阻力已提供大量自然阻尼），D 到 0 后再增大 P |
| 出现超调/振荡 | 阻尼不足 | 增大 D |
| **25 Hz 以上快速 P 项振荡** | 超过 Nyquist 极限（环率 50 Hz） | D 无法抑制 → P、I、D 全部减少约 **30%**；若 D 已为 0 仍振，只能把 P 再减 30%，即调参极限 |
- **整体增益**：PD 平衡确定后，把 P、I、D 一起按 **×1.2** 放大；因物理阻尼不随增益放大，通常需要比 P 更多地增加 D 来补偿，或整体放大后再回头微调 PD 平衡。
## 4. 完成与后续
- 完成标志：Angle 模式三轴角度跟踪幅值一致；Altitude 与 Direct Velocity(Position) 模式下速度跟踪快速且无振荡。
- 下一步：位置控制器与运动学路径（kinematic path）参数调参（Part 5）。
---
### 全程调参顺序速记
```
滤波器(Part 2) → Rate PID(Part 3) → Angle P(Part 4) → 垂直速度(Altitude) → 水平速度(Position) → 位置/路径
```
