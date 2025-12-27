---
title: ROS通信
createTime: 2025/12/27 14:44:18
permalink: /notes/ROS/uhdwbzqj/
---
## ROS通信

### 话题通信

**流程:**

1. 编写发布方实现；
2. 编写订阅方实现；
3. 编辑配置文件；
4. 编译并执行。

#### 1.发布方

>     实现流程:
>         	1.包含头文件 
>         	2.初始化 ROS 节点:命名(唯一)
>         	3.实例化 ROS 句柄
>         	4.实例化 发布者 对象
>         	5.组织被发布的数据，并编写逻辑发布数据

```cpp
// 1.包含头文件 
#include "ros/ros.h"
#include "std_msgs/String.h" //普通文本类型的消息
#include <sstream>

int main(int argc, char  *argv[])
{   
    //设置编码
    setlocale(LC_ALL,"");

    //2.初始化 ROS 节点:命名(唯一)
    // 参数1和参数2 后期为节点传值会使用
    // 参数3 是节点名称，是一个标识符，需要保证运行后，在 ROS 网络拓扑中唯一
    ros::init(argc,argv,"talker");
    //3.实例化 ROS 句柄
    ros::NodeHandle nh;//该类封装了 ROS 中的一些常用功能

    //4.实例化 发布者 对象
    //泛型: 发布的消息类型
    //参数1: 要发布到的话题
    //参数2: 队列中最大保存的消息数，超出此阀值时，先进的先销毁(时间早的先销毁)
    ros::Publisher pub = nh.advertise<std_msgs::String>("chatter",10);

    //5.组织被发布的数据，并编写逻辑发布数据
    //数据(动态组织)
    std_msgs::String msg;
    // msg.data = "你好啊！！！";
    std::string msg_front = "Hello 你好！"; //消息前缀
    int count = 0; //消息计数器

    //逻辑(一秒10次)
    ros::Rate r(10);

    //节点不死
    while (ros::ok())
    {
        //使用 stringstream 拼接字符串与编号
        std::stringstream ss;
        ss << msg_front << count;
        msg.data = ss.str();
        //发布消息
        pub.publish(msg);
        //加入调试，打印发送的消息
        ROS_INFO("发送的消息:%s",msg.data.c_str());

        //根据前面制定的发送贫频率自动休眠 休眠时间 = 1/频率；
        r.sleep();
        count++;//循环结束前，让 count 自增
        //官方建议加上
        ros::spinOnce();
    }


    return 0;
}
```

- 句柄是啥?

  在 ROS 里，`NodeHandle` 就是你操作 ROS 资源（话题、服务、参数）的入口。

- `std_msgs::String` 和 `std::string`的关系

  | 类型               | 来源      | 用途                    | 内部字段              |
  | :----------------- | --------- | ----------------------- | --------------------- |
  | `std::string`      | C++标准库 | 纯 C++ 字符串处理       | N/A                   |
  | `std_msgs::String` | ROS消息库 | 在 ROS 话题中发送字符串 | `data`（std::string） |

	在 ROS 中发布字符串消息 **必须用 std_msgs::String**，但可以先用 std::string 拼接好内容，然后赋值给 msg.data

-  `ss.str()`

    ```
    std::stringstream ss;
    ss << msg_front << count;
    msg.data = ss.str();  // 这里
    ```

	ss是一个stringstream对象，ss.str()返回的是std::string类型的，直接可以赋值给msg.data（也是std::string类型）。

- `msg.data.c_str()`

  ```
  ROS_INFO("发送的消息:%s", msg.data.c_str());
  ```

  msg.data是一个std::string，但是ROS_INFO是c风格的格式化输出（类似printf），所有要用c_str()将msg.data的类型转换成const char*。

  

#### 2.订阅方

>     实现流程:
>         	1.包含头文件 
>        	 2.初始化 ROS 节点:命名(唯一)
>        	 3.实例化 ROS 句柄
>         	4.实例化 订阅者 对象
>         	5.处理订阅的消息(回调函数)
>         	6.设置循环调用回调函数

```cpp
#include "ros/ros.h"
#include "std_msgs/String.h"

// 回调函数
void doMsg(const std_msgs::String::ConstPtr &msg)
{
    // 通过msg获取并操作订阅到的数据
    ROS_INFO("翠花订阅的数据：%s", msg->data.c_str())
}

int main(int argc, char *argv[])
{
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "cuihua");
    ros::NodeHandle nh;
    // 第一个参数是话题，第二个是队列容量，第三个是回调函数
    ros::Subscriber sub = nh.subscribe("fang", 10, doMsg);
	// spin的作用是让节点进入一个循环监听状态，持续接收消息并调用对应的回调函数。
    ros::spin();
    
    return 0;
}
```

- 回调函数是什么

  回调函数（callback function）是 **程序在特定事件发生时自动调用的函数**。在 ROS 里，它是订阅者处理消息的核心机制。`ros::spin()` 会不断检查消息队列，一旦 `"fang"` 有新消息，ROS 自动调用 `doMsg(msg)`

#### 3.自定义msg

##### 	1.在包里面新建一个msg文件夹，在文件夹里面创建一个msg后缀的文件，并编辑

例如：

```
string name
int32 age
float32 height
```

##### 	2. 修改 package.xml

加上两行：

```xml
<build_depend>message_generation</build_depend>
<exec_depend>message_runtime</exec_depend>
```

- `message_generation` 用于 **编译阶段**（生成消息头文件）

- `message_runtime` 用于 **运行阶段**（运行时依赖）

##### 	3.修改CmakeLists.txt

```cmake
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation # 需要加入 message_generation,必须有 std_msgs
)

## 配置 msg 源文件
add_message_files(
  FILES
  # 你的 msg 文件，比如 Person.msg
)

# 这一段解开注释
generate_messages(
  DEPENDENCIES
  std_msgs
)

# 执行时依赖
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES demo02_talker_listener
  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
#  DEPENDS system_lib
)
```

##### 	4.编译

编译之后，C++ 生成调用的中间文件(.../工作空间/devel/include/包名/xxx.h)，后续调用相关 msg 时，是从这些中间文件调用的。

#### 4. 自定义msg的使用

##### 	0.修改c_cpp_properties.json

"/xxx/yyy工作空间/devel/include/**" //配置 head 文件的路径 ，这一行记得改

```json
{
    "configurations": [
        {
            "browse": {
                "databaseFilename": "",
                "limitSymbolsToIncludedHeaders": true
            },
            "includePath": [
                "/opt/ros/noetic/include/**",
                "/usr/include/**",
                "/xxx/yyy工作空间/devel/include/**" //配置 head 文件的路径 
            ],
            "name": "ROS",
            "intelliSenseMode": "gcc-x64",
            "compilerPath": "/usr/bin/gcc",
            "cStandard": "c11",
            "cppStandard": "c++17"
        }
    ],
    "version": 4
}
```

##### 	1.发布方

```cpp
#include "ros/ros.h"
#include "plumbing_pub_sub/Person.h" // 新加入的


int main(int argc, char *argv[])
{
    setlocale(LC_ALL, "");
    ROS_INFO("这是消息的发布方：");
    ros::init(argc, argv, "laoshi");
    ros::NodeHandle nh;
    
    // 这里也注意
    ros::Publisher pub = nh.advertise<plumbing_pub_sub::Person>("liaoTian", 10);
    plumbing_pub_sub::Person person;
    person.name = "章三";
    person.age = 1;
    person.height = 1.7;

    ros::Rate rate(1);
    while(ros::ok())
    {
        person.age += 1;
        // 核心
        pub.publish(person);
        ROS_INFO("发布的消息：%s, %d, %.2f", person.name.c_str(), person.age, person.height);
        rate.sleep();
        ros::spinOnce();
    }

    return 0;
}
```

##### 	2.订阅方

```cpp
#include "ros/ros.h"
#include "plumbing_pub_sub/Person.h"

void doPerson(const plumbing_pub_sub::Person::ConstPtr& person)
{
    ROS_INFO("订阅方的信息：%s, %d, %.2f", person->name.c_str(), person->age, person->height);
}

int main(int argc, char *argv[])
{
    setlocale(LC_ALL, "");
    ROS_INFO("这里是订阅方：");
    ros::init(argc, argv, "xueshen");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("liaoTian", 10, doPerson);
    ros::spin();

    return 0;
}
```

##### 	3.配置CMakeLists.txt

需要添加 **add_dependencies** 用以设置所依赖的消息相关的中间文件。

```cmake
add_executable(person_talker src/person_talker.cpp)
add_executable(person_listener src/person_listener.cpp)

add_dependencies(person_talker ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(person_listener ${PROJECT_NAME}_generate_messages_cpp)


target_link_libraries(person_talker
  ${catkin_LIBRARIES}
)
target_link_libraries(person_listener
  ${catkin_LIBRARIES}
){
	"version": "2.0.0",
	"tasks": [
		{
			"type": "cmake",
			"label": "CMake: 生成",
			"command": "build",
			"targets": [
				"[N/a - 选择工具包]"
			],
			"group": "build",
			"problemMatcher": [],
			"detail": "CMake 模板 生成 任务"
		}
	]
}
```

### 服务通信

#### 自定义srv

**1.创建srv文件**

服务通信中，数据分成两部分，**请求**与**响应**，在 srv 文件中请求和响应**使用`---`分割**，具体实现如下:

功能包下新建 srv 目录，在srv下添加 xxx.srv 文件，内容:

```
# 客户端请求时发送的两个数字
int32 num1
int32 num2
---
# 服务器响应发送的数据
int32 sum
```

**2.编辑配置文件**

**package.xml**中添加编译依赖与执行依赖

```
  <build_depend>message_generation</build_depend>
  <exec_depend>message_runtime</exec_depend>
```

**CMakeLists.txt**编辑 srv 相关配置

```
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)
# 需要加入 message_generation,必须有 std_msgs
```

```
add_service_files(
  FILES
  AddInts.srv # 自己创建的srv
)
```

```
generate_messages(
  DEPENDENCIES
  std_msgs
)
```

**3.编译**

编译后的中间文件查看:

C++ 需要调用的中间文件(.../工作空间/devel/include/包名/xxx.h)

**4.TIPS**

将".../工作空间/devel/include/**"这个路径写到.vscode的c_cpp_properties.json的"includePath"里面

#### 2.服务端

```cpp
#include "ros/ros.h"
#include "service/Addints.h" // 这是自定义srv产生的头文件地址

/*
    服务端实现：解析客户端的数据，并产生响应
        1.包含头文件
        2.初始化ros节点
        3.创建句柄
        4.创建服务对象
        5.处理请求并响应
        6.spin()
*/

bool doNums(service::Addints::Request &request, 
            service::Addints::Response &response)
{
    // 处理请求
    int num1 = request.num1;
    int num2 = request.num2;
    ROS_INFO("收到的数据为%d,%d", num1, num2);
    // 组织响应
    int num = num1 + num2;
    response.sum = num;
    ROS_INFO("求和结果是:%d", num);

    return 1;
}

int main(int argc, char *argv[])
{
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "srv");
    ros::NodeHandle nh;
    // 这里的回调函数返回类型是bool
    ros::ServiceServer server = nh.advertiseService("add", doNums);
    ROS_INFO("服务器端启动：");
    ros::spin();
    return 0;
}
```

配置的时候记得还要把

`add_dependencies(server ${PROJECT_NAME}_gencpp)`这一行改一下，因为用到了自定义消息

#### 3.客户端

```cpp
#include "ros/ros.h"
#include "service/Addints.h"
/*
    客户端：提交两个整数，并处理响应结果
        1.头文件
        2.初始化ros节点
        3.创建节点句柄
        4.创建一个客户端对象
        5.提交请求并处理响应
        6.无需回调函数

    实现参数的动态提交：
        1.格式： rosrun 包名 程序名 1 2
        2.节点执行时需要获取命令中的参数，并组织进request

*/


int main(int argc, char *argv[])
{
    setlocale(LC_ALL, "");
    // 获取命令中的参数，必须是三个参数：文件名 + 两个整数
    if(argc != 3)
    {
        ROS_INFO("提交的参数个数不对！");
        return 1;
    }


    ros::init(argc, argv, "client");
    ros::NodeHandle nh;
    ros::ServiceClient client = nh.serviceClient<service::Addints>("add");

    service::Addints ai;
    // 提交请求，从参数中获取，参数是字符串格式，要转换成整型
    ai.request.num1 = atoi(argv[1]);
    ai.request.num2 = atoi(argv[2]);
    // 处理响应
    bool flag = client.call(ai);
    if(flag)
    {
        ROS_INFO("响应成功！");
        // 获取结果
        ROS_INFO("响应结果：%d", ai.response.sum);
    }
    else
        ROS_INFO("处理失败！");
    
    return 0;
}
```

**注意：**服务通信必须先启动服务端，再启动客户端

**但是，如果要先启动客户端：**

​	在客户端发送请求前添加:`client.waitForExistence();`

​	或:`ros::service::waitForService("AddInts");`

​	这是一个阻塞式函数，只有服务启动成功后才会继续执行

​	此处可以使用 launch 文件优化，但是需要注意 args 传参特点

### 参数服务器

> 参数服务器在ROS中主要用于实现不同节点之间的数据共享。参数服务器相当于是独立于所有节点的一个公共容器，可以将数据存储在该容器中，被不同的节点调用，当然不同的节点也可以往其中存储数据。
>
> 在 C++ 中实现参数服务器数据的增删改查，可以通过两套 API 实现:
>
> - ros::NodeHandle
> - ros::param

#### 增、改

```cpp
#include "ros/ros.h"

/*
    设置机器人的共享参数，类型，半径
        再修改半径
    添加参数的实现：
        ros::NodeHandle
            setParam("键", "值")
        ros::param
            set("键", "值")
    修改参数的实现：
        直接覆盖
*/

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "set_param");
    ros::NodeHandle nh;
    
    // 添加参数
        //方法一
    nh.setParam("type", "xiaohuang");
    nh.setParam("radius", 0.15);
        //方法二
    ros::param::set("type_param", "xiaobai");
    ros::param::set("radius_parm", 0.15);

    // 修改参数，就是直接覆盖
        //方法一
    nh.setParam("radius", 0.2);
        //方法二
    ros::param::set("radius_param", 0.2);
    
    return 0;
}
```

#### 查

```cpp
#include "ros/ros.h"

/*
    查询参数：
    在 roscpp 中提供了两套 API 实现参数操作
    ros::NodeHandle

        param(键,默认值) 
            存在，返回对应结果，否则返回默认值

        getParam(键,存储结果的变量)
            存在,返回 true,且将值赋值给参数2
            若果键不存在，那么返回值为 false，且不为参数2赋值

        getParamCached键,存储结果的变量)--提高变量获取效率
            存在,返回 true,且将值赋值给参数2
            若果键不存在，那么返回值为 false，且不为参数2赋值

        getParamNames(std::vector<std::string>)
            获取所有的键,并存储在参数 vector 中 

        hasParam(键)
            是否包含某个键，存在返回 true，否则返回 false

        searchParam(参数1，参数2)
            搜索键，参数1是被搜索的键，参数2存储搜索结果的变量

    ros::param ----- 与 NodeHandle 类似
*/

int main(int argc, char *argv[])
{
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "get_param");
    ros::NodeHandle nh;

    // ros::NodeHandle ----------------------------------------------------

    // param
    double radius = nh.param("radius", 0.5); //0.5是默认值，查不到则返回
    ROS_INFO("radius = %.2f", radius);

    // getParam
    // 存在,返回 true,且将值赋值给参数2
    // 若果键不存在，那么返回值为 false，且不为参数2赋值
    double radius2 = 0;
    bool result = nh.getParam("radius", radius2);
    if (result)
        ROS_INFO("radius = %.2f", radius2);
    else
        ROS_INFO("查询的变量不存在");

    // getParamCached，与getParam基本上一样，只是性能上略有提升

    // getParamNames(std::vector<std::string>)
    //  获取所有的键,并存储在参数 vector 中 
    std::vector<std::string> names;
    nh.getParamNames(names);
    for (auto &&name : names)
    {
        ROS_INFO("遍历的元素：%s", name.c_str());
    }

    // hasParam(键)
    // 否包含某个键，存在返回 true，否则返回 false
    bool result1 = nh.hasParam("radius");
    bool result2 = nh.hasParam("raddddddus");
    ROS_INFO("result1 = %d, result2 = %d", result1, result2);
    
    // searchParam()
    // 专门搜索键，参数1是被搜索的键，参数2存储搜索结果的变量
    std::string key;
    nh.searchParam("radius", key);
    ROS_INFO("搜索结果是：%s", key.c_str());

    // ros::param ----------------------------------------------------
    double radius_param = ros::param::param("radius", 0.5);

    std::vector<std::string> names_param;
    ros::param::getParamNames(names_param);
    
    //都类似
    return 0;
}

```

#### 删

```cpp
#include "ros/ros.h"
/*
    删除：

*/

int main(int argc, char *argv[])
{
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "del_param");
    ros::NodeHandle nh;

    // NodeHandle
    bool flag1 = nh.deleteParam("radius");
    if (flag1)
        ROS_INFO("删除成功");
    else
        ROS_INFO("删除失败");
    
    // ros::param
    bool flag2 = ros::param::del("radius_param");
    if (flag2)
        ROS_INFO("删除成功");
    else
        ROS_INFO("删除失败");
    
    return 0;
}
```

