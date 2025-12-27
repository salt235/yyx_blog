---
title: ROS常用命令
createTime: 2025/12/27 14:44:18
permalink: /notes/ROS/cr3v5uii/
---
### 常用命令

#### 1.rosnode

rosnode 是用于获取节点信息的命令

```
rosnode ping 节点名      测试到节点的连接状态
rosnode list            列出活动节点
rosnode info 节点名   	  打印节点信息
rosnode machine 机器名   列出指定设备上节点
rosnode kill  节点名 	  杀死某个节点
rosnode cleanup         清除不可连接的节点
```

#### 2.rostopic

**rostopic**包含rostopic命令行工具，用于显示有关ROS 主题的调试信息，包括发布者，订阅者，发布频率和ROS消息。它还包含一个实验性Python库，用于动态获取有关主题的信息并与之交互。

```
rostopic bw     显示主题使用的带宽
rostopic delay  显示带有 header 的主题延迟
rostopic echo   打印消息到屏幕
rostopic find   根据类型查找主题
rostopic hz     显示主题的发布频率
rostopic info   显示主题相关信息
rostopic list   显示所有活动状态下的主题
rostopic pub    将数据发布到主题
rostopic type   打印主题类型
```

- rostopic pub

可以直接调用命令向订阅者发布消息（输入rostopic pub 后按tab自动补全）

```
rostopic pub /主题名称 消息类型 消息内容
rostopic pub /chatter std_msgs gagaxixi
```

```
rostopic pub -r 10 /主题名称 消息类型 消息内容
```

#### 3.rosservice 

rosservice包含用于列出和查询ROS[Services](http://wiki.ros.org/Services)的rosservice命令行工具。

调用部分服务时，如果对相关工作空间没有配置 path，需要进入工作空间调用 source ./devel/setup.bash

```
rosservice args    打印服务参数
rosservice call    使用提供的参数调用服务
rosservice find    按照服务类型查找服务
rosservice info    打印有关服务的信息
rosservice list    列出所有活动的服务
rosservice type    打印服务类型
rosservice uri     打印服务的 ROSRPC uri
```

- rosservice call

调用服务，充当一个客户端来直接使用服务。

#### 4.rosmsg（用于话题通信）

rosmsg是用于显示有关 ROS消息类型的 信息的命令行工具。

```
rosmsg show    显示消息描述
rosmsg info    显示消息信息
rosmsg list    列出所有消息
rosmsg md5    显示 md5 加密后的消息
rosmsg package    显示某个功能包下的所有消息
rosmsg packages    列出包含消息的功能包
```

- rosmsg show

显示消息描述

```
//rosmsg show 消息名称
rosmsg show turtlesim/Pose
结果:
float32 x
float32 y
float32 theta
float32 linear_velocity
float32 angular_velocity
```

#### rossrv（用于服务通信）

rossrv是用于显示有关ROS服务类型的信息的命令行工具，与 rosmsg 使用语法高度雷同。

- rossrv list（用之前记得刷新环境变量）

```bash
yyx@TX:~/demo02_ws$ source ./devel/setup.bash 
yyx@TX:~/demo02_ws$ rossrv list | grep -i Addints
service/Addints
```

```
yyx@TX:~/demo02_ws$ rossrv info service/Addints
int32 num1
int32 num2
---
int32 sum
```

#### rosparam（用于参数服务器）

rosparam包含rosparam命令行工具，用于使用YAML编码文件在参数服务器上获取和设置ROS参数。

```
rosparam set    设置参数
rosparam get    获取参数
rosparam load    从外部文件加载参数
rosparam dump    将参数写出到外部文件
rosparam delete    删除参数
rosparam list    列出所有参数
```
- rosparam set
```
rosparam set name huluwa

//再次调用 rosparam list 结果
/name
/rosdistro
/roslaunch/uris/host_helloros_virtual_machine__42911
/rosversion
/run_id
```
- rosparam get
```
rosparam get name

//结果
huluwa
```

