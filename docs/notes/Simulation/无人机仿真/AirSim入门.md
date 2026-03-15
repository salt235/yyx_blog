---
title: AirSim入门
createTime: 2026/03/11 14:09:11
permalink: /notes/Simulation/g68l52i2/
---

飞控先用AirSim自带的`SimpleFlight`，后续对接到`px4`便于实机部署。

## 修改json文件

在` C:\Users\yyx_x\Documents\AirSim\settings.json`，修改文件：

```json
{
  "SettingsVersion": 1.2,
  "SimMode": "Multirotor",
  "Vehicles": {
    "Drone1": {
      "VehicleType": "SimpleFlight",
      "AutoCreate": true
    }
  }
}
```

这相当于一个配置表，AirSim 在 UE 场景启动时会自动读取这个文件。它启动了无人机仿真，创建了名为`Drone1`的无人机，并使用`SimpleFlight`飞控，这里没有用`px4`飞控，仅仅是入门环节。

## Python控制流程

```
Python
↓
AirSim RPC
↓
Drone1(SimpleFlight)
↓
无人机运动
```

## hello_drone.py

第一个示例脚本（起飞拍照）

```python
import airsim
import os
import numpy as np

client = airsim.MultirotorClient() # 连接到airsim
client.confirmConnection() # 确认连接，返回connected
client.enableApiControl(True) # 开启api
client.armDisarm(True) # 开启电机

client.takeoffAsync().join() # 起飞，join()是等待,起飞完成再继续执行
client.moveToPositionAsync(-1, 1, -1, 1).join() # 移动到指定位置

# 获取图像的api
responses = client.simGetImages([
    airsim.ImageRequest("0", airsim.ImageType.DepthVis, False, False),
    airsim.ImageRequest("1", airsim.ImageType.DepthPlanar, True, False)
])

print(f"Retrieved images: {len(responses)}")

# 保存两种图像
for i, response in enumerate(responses):
    if response.pixels_as_float:
        print(f"Float image size {len(response.image_data_float)}")
        img1d = np.array(response.image_data_float, dtype=np.float32)
        img2d = img1d.reshape(response.height, response.width)
        filename = os.path.join(os.getcwd(), f"py{i}.pfm")
        airsim.write_pfm(filename, img2d)
        print("Saved:", filename)
    else:
        print(f"Uint8 image size {len(response.image_data_uint8)}")
        filename = os.path.join(os.getcwd(), f"py{i}.png")
        airsim.write_file(filename, re
                          # 保存两种图像sponse.image_data_uint8)
        print("Saved:", filename)
```

## 一些常用的API

详情见：https://frendowu.github.io/AirSim-docs-zh/apis/#common-apis

## 常见问题

### 1. 运行 API 时，Unreal 引擎运行速度明显变慢。

如果发现虚幻引擎窗口失去焦点时，虚幻引擎运行速度明显变慢，则在虚幻编辑器中转到“编辑->编辑器首选项”，在“搜索”框中键入“CPU”，并确保“在后台运行时减少 CPU 使用率”未选中。
