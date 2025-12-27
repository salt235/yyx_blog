---
title: ROS运行管理
createTime: 2025/12/27 14:44:18
permalink: /notes/ROS/787ihz5n/
---
## ROS运行管理

### 元功能包

> 在ROS中，提供了一种方式可以将不同的功能包打包成一个功能包，当安装某个功能模块时，直接调用打包后的功能包即可，该包又称之为元功能包(metapackage)。

**首先:**新建一个功能包

**然后:**修改**package.xml** ,内容如下:

```**xml
 <exec_depend>被集成的功能包</exec_depend>
 .....
 <export>
   <metapackage />
 </export>
```

**最后:**修改 CMakeLists.txt,内容如下:

```
cmake_minimum_required(VERSION 3.0.2)
project(demo)
find_package(catkin REQUIRED)
catkin_metapackage()
// 不可以有换行
```

### launch文件

> 使用方法：
>
> ​	roslaunch 包名 launch文件名.launch

- launch标签

  ``` 
  <launch deprecated = "弃用声明">
  ------
  </launch>
  ```

- node标签

  属性：

  - pkg="包名"

    节点所属的包

  - type="nodeType"

    节点类型(与之相同名称的可执行文件)

  - name="nodeName"

    节点名称(在 ROS 网络拓扑中节点的名称)

  - args="xxx xxx xxx" (可选)

    将参数传递给节点

  - machine="机器名"

    在指定机器上启动节点

  - respawn="true | false" (可选)

    如果节点退出，是否**自动重启**

  - respawn_delay=" N" (可选)

    如果 respawn 为 true, 那么延迟 N 秒后启动节点

  - required="true | false" (可选)

    该节点是否必须，如果为 true,那么如果该节点退出，将杀死**整个** roslaunch

  - ns="xxx" (可选)

    在指定命名空间 xxx 中启动节点

  - clear_params="true | false" (可选)

    在启动前，删除节点的私有空间的所有参数

  - output="log | screen" (可选)

    日志发送目标，可以设置为 log 日志文件，或 screen 屏幕,默认是 log

- include标签

  `include`标签用于将另一个 xml 格式的 launch 文件导入到当前文件

  - file="$(find 包名)/xxx/xxx.launch"

    要包含的文件路径

  - ns="xxx" (可选)

    在指定命名空间导入文件

  在launch文件夹中新建一个start_turtle_use.launch，来复用之前写的start_turtle.launch。

  ```
  <!-- 复用start_turtle.launch -->
  <launch>
      <include file ="$(find test)/launch/start_turtle.launch" />
  </launch>  
  ```
  
- remap标签

  用于话题重命名

  这里把乌龟GUI的控制话题/turtle1/cmd_vel重命名/cmd_vel。

  ```
  <!-- 启动乌龟GUI与键盘控制节点 -->
  <launch>
      <!-- 乌龟GUI -->
      <node pkg="turtlesim" type="turtlesim_node" name="turtle1" output="screen">
          <remap from = "/turtle1/cmd_vel" to = "/cmd_vel"/>;
      </node>
      <!-- 乌龟键盘控制 -->
      <node pkg="turtlesim" type="turtle_teleop_key" name="key" output="screen" />
  </launch>
  ```

  然后可以调用ros自带的键盘控制节点，他的话题就是cmd_vel。

  ```bash
  yyx@TX:~$ rosrun teleop_twist_keyboard teleop_twist_keyboard.py
  ```

- param标签

  `<param>`标签主要用于在参数服务器上设置参数，参数源可以在标签中通过 value 指定，也可以通过外部文件加载，在`<node>`标签中时，相当于私有命名空间。

  - name="命名空间/参数名"

    参数名称，可以包含命名空间

  - value="xxx" (可选)

    定义参数值，如果此处省略，必须指定外部文件作为参数源

  - type="str | int | double | bool | yaml" (可选)

    指定参数类型，如果未指定，roslaunch 会尝试确定参数类型，规则如下:

    - 如果包含 '.' 的数字解析未浮点型，否则为整型
    - "true" 和 "false" 是 bool 值(不区分大小写)
    - 其他是字符串

  ```
  <launch>
      <param name = "param-a" type = "int" value = "100"/>
      <!-- 乌龟GUI -->
      <node pkg="turtlesim" type="turtlesim_node" name="turtle1" output="screen" >
          <param name = "param-b" type = "int" value = "100"/>
      </node>
      <!-- 乌龟键盘控制 -->
      <node pkg="turtlesim" type="turtle_teleop_key" name="key" output="screen" />
  </launch>
  ```

  这里加了/param-a和/turtle1/param-b两个

  参数。在launch标签和node标签内的param有所区别。

- rosparam

  略。

- group

  `<group>`标签可以对节点分组，具有 ns 属性，可以让节点归属某个命名空间。

  略。

- arg

  `<arg>`标签是用于动态传参，类似于函数的参数，可以增强launch文件的灵活性。
  
  有点像宏定义，可以设置参数的默认值。
  
  略。

### 重名问题

#### 节点重名

- 语法: rosrun 包名 节点名 __ns:=新名称，可以设置节点的命名空间。

    ```bash
    rosrun turtlesim turtlesim_node __ns:=/xxx
    ```

- 语法: rosrun 包名 节点名 __name:=新名称

    ```bash
    rosrun turtlesim  turtlesim_node __name:=t1
    ```

- 语法: rosrun 包名 节点名 ns:=新名称 name:=新名称

    ```bash
    rosrun turtlesim turtlesim_node __ns:=/xxx __name:=tn
    ```

- 在launch文件里面也可以起名字和命名空间

#### 话题重名

- 将 teleop_twist_keyboard 节点的话题由/cmd_vel设置为/turtle1/cmd_vel

    ```
    rosrun teleop_twist_keyboard teleop_twist_keyboard.py /cmd_vel:=/turtle1/cmd_vel
    ```

- 通过launch文件的remap
- 编码设置话题名称（略）

#### 参数名称设置

- rosrun设置

  **语法:** rosrun 包名 节点名称 _参数名:=参数值

  ```
  rosrun turtlesim turtlesim_node _A:=100
  ```

- launch文件设置

  param标签

- 编码实现

  略。
