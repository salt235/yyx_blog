---
title: "PX4 调参 Part 2：陀螺仪滤波"
createTime: 2026/09/16 00:00:00
permalink: /notes/UAV/px4/tuning/part-2-gyro-filtering.html
---

# PX4 调参 Part 2：陀螺仪滤波
> 来源：[PX4 Ultimate Tuning Guide - Part 2 Gyro Filtering](https://www.youtube.com/watch?v=PQOXJpstZx4&t=1154s)（Chris Rosser）
> 前置：[PX4 调参 Part 1 装机与基础设置流程总结](/notes/UAV/px4/tuning/part-1-setup.html)
> 相关：[PX4 滤波与 PID 调参](/notes/UAV/px4/pid-tuning.html)、[PX4 多旋翼四个控制环](/notes/UAV/px4/control-loops.html)
> 目标：在**噪声抑制最大**与**滤波延迟最小**之间取平衡。滤波器是 PID 的地基，顺序为 滤波器 → Acro(Rate) → Stabilize → Altitude → Position → 自主导航。
## 1. 准备
1. **换快卡**：用 SanDisk Extreme 32 GB microSDHC（小文件写入最快，减少日志掉点）；PX4 目前不支持大于 32 GB 的卡。
2. **设日志参数**（QGC → Vehicle Configuration → Parameters）：
   - `IMU_GYRO_RATEMAX`：7 寸及以上设 800 Hz，5 寸及以下设约 1000 Hz（桨小转速高，需要更多高频信息）。
   - `SD_LOG_PROFILE`：在 default set 之外，再勾选 **high rate** 和 **raw FIFO high rate IMU gyro**。
   - 保存后 **Reboot Vehicle**。
3. **安全前提**：电机/桨方向正确（炸机最常见原因）、失控保护开关（kill switch）已验证可用。
## 2. 悬停测试
- 用 **Stabilized 模式**（油门直接控制，松杆自动回水平）。
- 有条件用测试台；否则在开阔、下方有软草/网的场地飞。
- 解锁后先在地面轻微前后左右倾斜，确认各通道响应方向正确。
- 起飞悬停 **20–30 秒**即可，降落上锁。
## 3. 下载并分析日志
1. USB 连 QGC → Analyze Tools → Log Download → Refresh → 选本次日志 → Download。
2. 上传到 **logs.px4.io**（Flight Review），访问权限选 **share by link**（日志含 GPS 位置信息）。
3. 看 **Angular Velocity Power Spectral Density** 图：
   - 横轴上限约为 `IMU_GYRO_RATEMAX` 的 0.45 倍；若看不到预期的电机噪声，说明采样率不够，需调高该参数。
   - 电机基频 = 电机 RPM ÷ 60；图上可见基频及 2、3 次谐波。
   - 两叶桨在 2 倍频有明显噪声带；三叶桨在 3 倍频更强、2 倍频较弱（桨叶通过机臂的频率）。
## 4. 动态陷波（Notch）滤波
### 4.1 取 RPM 数据
- **首选双向 DShot**：`DSHOT_BIDIR_EN` = Enabled；`MOT_POLE_COUNT` = 转子磁铁数（小电机常见 14，很小的 12，大电机常见 28），用于把电调上报的电 RPM 换算成机械 RPM。频率最高、延迟最低。
- **备选 ESC 遥测线**：把所有电调遥测线并到同一个 UART 的 RX 脚，再把 `DSHOT_TEL` 设为该 UART。（视频中 Matek/“Miko Air 743 V2” 的 UART7 有 bug 无法用于 ESC 遥测。）
- **标定转速范围**：在 Position/Altitude 模式做爬升/下降飞行，记录常用**最大/最小电机 RPM**（视频示例：最大约 11000 RPM，最小约 5600 RPM）。
### 4.2 设置动态陷波
| 参数                 | 设置方法                                                           |
| ------------------ | -------------------------------------------------------------- |
| `IMU_GYRO_DNF_EN`  | 设为 1（仅用 ESC RPM）；若勾了 FFT 要取消                                   |
| `IMU_GYRO_FFT_EN`  | 关闭（FFT 引擎本系列不讲，更适合没有 RPM 数据的大机型）                               |
| `IMU_GYRO_DNF_MIN` | = 最低电机频率 × 0.9 = (最低 RPM ÷ 60) × 0.9；例 5600÷60×0.9 ≈ **84 Hz** |
| `IMU_GYRO_DNF_BW`  | 初值取 DNF_MIN 的约 20%（例 ≈17 Hz），之后再压窄                             |
| `IMU_GYRO_DNF_HMC` | 两叶桨设 2，三叶桨设 3；普通机型设 4 以上没有收益                                   |
说明：动态陷波 Q 值恒定，频率升高时带宽同比变宽，正好覆盖转速变化引起的噪声漂移。
 验证：看 raw angular velocity gyro 曲线，收窄带宽后曲线应基本不变；若突然出现高频“毛刺”，说明陷波两侧漏噪声，需把带宽稍微加大。
## 5. 陀螺仪低通滤波
- 原理：机体真实转动一般在 **30 Hz 以下**，振动多在 **50 Hz 以上**（大桨低转速时振动频率会偏低）；截止频率越低噪声越干净，但延迟越大，延迟会拖慢对气流和打杆的响应。
- 目标：在保证高频噪声被压住的前提下，把截止频率**尽量提高**。
- 从 Angular Velocity FFT 图找“低频真实运动峰 → 安静区 → 电机基频峰”，把 `IMU_GYRO_CUTOFF` 设在**第一个电机频率峰的左侧**（示例 80 Hz）。
- `IMU_DGYRO_CUTOFF` 始终 = `IMU_GYRO_CUTOFF` 的 **1/2**（示例 80/40；若陀螺 50→100，则 D 项 25→50）。两者必须同步改，否则可能出现响应很快但 D 延迟过大引起的自激振荡。
## 6. 迭代与完成标准
1. 每次只抬高一点截止频率（保持 2:1），飞一次、看一次日志。
2. 检查 **raw angular velocity gyro** 与 **Actuator Controls FFT**：一旦出现明显毛刺，或执行器 30 Hz 以上噪声明显增加，就停止继续抬高（会让电机发热、加重电调负担）。
3. 反复几次，直到找到噪声抑制与延迟的最佳平衡。
4. 完成标志：陷波精准罩住全部电机噪声 + 低通截止尽量高且无漏噪声 → 进入下一阶段 Rate PID 调参。
