---
title: ProjectAirSim - UE5 - PX4部署（linux）
createTime: 2026/03/15 13:15:38
permalink: /notes/Simulation/5fxuxeix/
---
# ProjectAirSim - UE5 - PX4部署（linux）

## 部署PX4

```bash
# 首先自行配置好github的ssh公钥
git clone git@github.com:PX4/PX4-Autopilot.git
cd PX4-Autopilot
git checkout v1.12.3
bash Tools/setup/ubuntu.sh # 安装依赖
make px4_sitl none_iris

# 下面是错误1
# /data/huangth/PX4-Autopilot/platforms/common/px4_work_queue/WorkQueueManager.cpp:274:95: error: no matching function for call to ‘max(long int, int)’
#   274 |    const size_t stacksize_adj = math::max(PTHREAD_STACK_MIN, PX4_STACK_ADJUSTED(wq->stacksize));
#       |                                                                                               ^
# compilation terminated due to -Wfatal-errors.
# [583/808] Building CXX object src/mod...s__mavlink.dir/mavlink_messages.cpp.o
# ninja: build stopped: subcommand failed.
# make: *** [Makefile:233：px4_sitl] 错误 1

# 如果编码出现错误1，执行以下代码
#sed -i 's/math::max(PTHREAD_STACK_MIN/math::max((int)PTHREAD_STACK_MIN/g' \
#  /data/<user-name>/PX4-Autopilot/platforms/common/px4_work_queue/WorkQueueManager.cpp
```

## 部署ProjectAirSim

```bash
git clone git@github.com:iamaisim/ProjectAirSim.git
cd ProjectAirSim
./setup_linux_dev_tools.sh
```

## 部署UE5

首先，参考https://dev.epicgames.com/documentation/en-us/unreal-engine/linux-development-quickstart-for-unreal-engine，绑定epic和github，然后再github中加入epic组织，之后就可以下载源码了。

```bash
# 下载下载UnrealEngine@5.2.0-release
git clone -b 5.2.0-release git@github.com:EpicGames/UnrealEngine.git
```

然后等待很长时间clone。

```bash
cd /UnrealEngine
./Setup.sh
```

然后等待很长时间下载依赖。

```bash
./Engine/Build/BatchFiles/RunUAT.sh BuildGraph \
    -target="Make Installed Build Linux" \
    -script=Engine/Build/InstalledEngineBuild.xml \
    -set:HostPlatformOnly=true \
    -set:WithLinuxArm64=false \
    -set:WithFullDebugInfo=false \
    -set:WithDDC=true
```

然后等待很长时间编译，这几步下来占用一百多G磁盘。

```bash
# 整理和注册编译好的 Unreal Engine 安装版。等 Unreal Engine 编译完成后，BuildGraph 会在源码目录里生成一个 Installed Build，把这个 UE 安装版移动到~/UE_installed_build。
mv ./LocalBuilds/Engine/Linux ~/UE_installed_build
cd ~/UE_installed_build
# 下面可以把源码删除了
rm -rf <UE source path>
# 设置环境变量
echo 'export UE_ROOT=~/UE_installed_build' >> ~/.bashrc
source ~/.bashrc
echo $UE_ROOT
```

## 运行

```bash
# 运行PX4-SITL
cd /data/<user-name>/PX4-Autopilot
make px4_sitl none_iris

# 运行ProjectAirSim中的block环境
/data/<user_name>/UE_installed_build/Engine/Binaries/Linux/UnrealEditor \
/data/<user_name>/ProjectAirSim-main/unreal/Blocks/Blocks.uproject

# 创建conda client环境
sudo apt install python-dev python-venv
conda create -n airsim pytho=3.8 -y
conda activate airsim
python -m pip install --upgrade pip
python -m pip install setuptools wheel
cd /data/huangth/ProjectAirSim-main
python -m pip install -e client/python/projectairsim

# 运行示例脚本
cd /data/<user_name>/ProjectAirSim-main/client/python/example_user_scripts
python hello_drone.py
```

