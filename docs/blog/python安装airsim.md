---
title: python安装airsim
createTime: 2026/01/06 10:42:59
tags:
    - 实践记录
    - 学习
---
## 目标

在 **Windows + Python 环境** 中成功使用 **AirSim 的 Python API**，用于无人机/车辆仿真控制。

### 试错

直接 `pip install airsim` 由于 PyPI 访问和证书问题始终无法拉取依赖；最终通过本地 AirSim 源码安装，并配合国内镜像，才成功绕过网络与 SSL 问题完成安装。

### 流程

- 首先下载源码：

- 根据https://github.com/microsoft/AirSim/issues/4920，修改一下`AirSim-1.8.1-windows\PythonClient`下面的`setup.py`

- conda新建`python3.8`的一个环境：

  ```bash
  conda create -n airsim python=3.8
  conda activate airsim
  cd 源码安装的目录\AirSim-1.8.1-windows\PythonClient
  # 记得关闭梯子
  pip install . -i https://pypi.tuna.tsinghua.edu.cn/simple
  ```

- 检测安装和常用api：

  ```bash
  (airsim_nav) PS D:\GoogleDownload\AirSim-1.8.1-windows\PythonClient> python
  Python 3.8.20 (default, Oct  3 2024, 15:19:54) [MSC v.1929 64 bit (AMD64)] :: Anaconda, Inc. on win32
  Type "help", "copyright", "credits" or "license" for more information.
  Ctrl click to launch VS Code Native REPL
  >>> import airsim
  >>> airsim.MultirotorClient
  <class 'airsim.client.MultirotorClient'>
  >>> airsim.CarClient
  <class 'airsim.client.CarClient'>
  ```

  

