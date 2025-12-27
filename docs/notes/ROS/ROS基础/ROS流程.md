---
title: ROS流程
createTime: 2025/12/27 14:44:18
permalink: /notes/ROS/acp786k8/
---
## ROS流程

### VSCode基本流程

ROS中的程序即便使用不同的编程语言，实现流程也大致类似，实现流程大致如下：

1. 先创建一个工作空间；
2. 再创建一个功能包；
3. 编辑源文件；
4. 编辑配置文件；
5. 编译并执行。

#### 1.创建工作空间并初始化

```
mkdir -p 自定义空间名称/src
cd 自定义空间名称
catkin_make
```

上述命令，首先会创建一个工作空间以及一个 src 子目录，然后再进入工作空间调用 catkin_make命令编译。

#### 2.启动vscode

```
cd 自定义空间名称
code .
```

#### 3.vscode 中编译 ros

快捷键 ctrl + shift + B 调用编译，选择 `catkin_make:build` 右边的齿轮，修改.vscode/tasks.json 文件。

把这一块替换进去。

```json
{
// 有关 tasks.json 格式的文档，请参见
    // https://go.microsoft.com/fwlink/?LinkId=733558
    "version": "2.0.0",
    "tasks": [
        {
            "label": "catkin_make:debug", //代表提示的描述性信息
            "type": "shell",  //可以选择shell或者process,如果是shell代码是在shell里面运行一个命令，如果是process代表作为一个进程来运行
            "command": "catkin_make",//这个是我们需要运行的命令
            "args": [],//如果需要在命令后面加一些后缀，可以写在这里，比如-DCATKIN_WHITELIST_PACKAGES=“pac1;pac2”
            "group": {"kind":"build","isDefault":true},
            "presentation": {
                "reveal": "always"//可选always或者silence，代表是否输出信息
            },
            "problemMatcher": "$msCompile"
        }
    ]
}
```

#### 4.创建ros功能包

选定 src 右击 ---> create catkin package

**设置包名：**例如 hello_vscode

**添加依赖：**例如 roscpp rospy std_msgs

这个时候再按 ctrl + shift + B 调用编译看看有没有问题。

#### 5.进入src右键新建cpp

举个例子

```c++
/*
    控制台输出 HelloVSCode !!!

*/
#include "ros/ros.h"

int main(int argc, char *argv[])
{
    setlocale(LC_ALL,"");
    //执行节点初始化
    ros::init(argc,argv,"HelloVSCode");

    //输出日志
    ROS_INFO("Hello VSCode!!!哈哈哈哈哈哈哈哈哈哈");
    return 0;
}
```

**PS1: 如果没有代码提示**

修改 .vscode/c_cpp_properties.json

设置 "cppStandard": "c++17"

**PS2: main 函数的参数不可以被 const 修饰**

**PS3: 当ROS__INFO 终端输出有中文时，会出现乱码**

解决办法：在函数开头加入下面代码的任意一句

```c++
setlocale(LC_CTYPE, "zh_CN.utf8");
setlocale(LC_ALL, "");
```

#### 6.配置 CMakeLists.txt

在src下面打开CMakeLists.txt，大概在第136行和第149行，修改一下

```cmake
add_executable(节点名称（一般就是源文件名）
  src/C++源文件名.cpp
)
    
target_link_libraries(节点名称
  ${catkin_LIBRARIES}
)
```

然后 ctrl + shift + B 调用编译。

#### 7.执行

新建第一个终端运行：

```bash
roscore
```

再新建一个终端：

```bash
source ./devel/setup.bash
rosrun 包名 节点名称
```





