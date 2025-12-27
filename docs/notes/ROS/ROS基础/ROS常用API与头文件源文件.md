---
title: ROS常用API与头文件源文件
createTime: 2025/12/27 14:44:18
permalink: /notes/ROS/19e5w4xs/
---
## ROS常用API与头文件源文件

### ROS::init()

```cpp
void init(int &argc, char **argv, const std::string& name, uint32_t options = 0);
\param argc 参数个数
\param argv 参数列表
\param name 节点名称，需要保证其唯一性，不允许包含命名空间
\param options 节点启动选项，被封装进了ros::init_options
```

- argc，argv的使用

  可以按照特定的格式给参数服务器传参。

  ```
  rosrun 包名 文件名 _参数名:=参数值
  ```

  ```bash
  yyx@TX:~/demo01_ws$ rosrun sub_pub pub _length:=2.0
  ```

  这个时候就可以看到参数服务器多了一个全局参数```/talker/length ```：

  ```bash
  yyx@TX:~/demo01_ws$ rosparam list
  /rosdistro
  /roslaunch/uris/host_tx__44263
  /rosversion
  /run_id
  /talker/length 
  ```

- option的使用

  ```cpp
  ros::init(argc, argv, "talker", ros::init_options::AnonymousName);
  ```

  这样就可以选择一个节点多次启动，会在节点名后面添加一个随机数字避免重复。

### NodeHandle

>  在 roscpp 中，话题和服务的相关对象一般由 NodeHandle 创建。

#### 话题发布者对象

```cpp
 Publisher advertise(const std::string& topic, uint32_t queue_size, bool latch = false)
```

```cpp
\param topic 发布消息使用的话题
\param queue_size 等待发送给订阅者的最大消息数量
\param latch (optional) 如果为 true,该话题发布的最后一条消息将被保存，并且后期当有订阅者连接时会将该消息发送给订阅者
```

- latch设置为true的作用？

  以静态地图发布为例，可以将地图发布对象的latch设置为true，并且只发布一次数据，每当有订阅者连接时，都会收到一次地图，提高了地图的发送效率。

### 回旋函数

> 在ROS程序中，频繁的使用了 ros::spin() 和 ros::spinOnce() 两个回旋函数，可以用于处理回调函数。

**相同点:**二者都用于处理回调函数；

**不同点:**ros::spin() 是进入了循环执行回调函数，而 ros::spinOnce() 只会执行一次回调函数(没有循环)，在 ros::spin() 后的语句不会执行到，而 ros::spinOnce() 后的语句可以执行。

### 时间

> ROS中时间相关的API是极其常用，比如:获取当前时刻、持续时间的设置、执行频率、休眠、定时器...都与时间相关。

#### 获取当前时刻

```cpp
ros::init(argc,argv,"hello_time");
ros::NodeHandle nh;//必须创建句柄，否则时间没有初始化，导致后续API调用失败
ros::Time right_now = ros::Time::now();//将当前时刻封装成对象
ROS_INFO("当前时刻:%.2f",right_now.toSec());//获取距离 1970年01月01日 00:00:00 的秒数
ROS_INFO("当前时刻:%d",right_now.sec);//获取距离 1970年01月01日 00:00:00 的秒数
```

#### 设置制定时刻

```cpp
ros::Time t1(20, 232323343);
ros::Time t2(20.6);
ROS_INFO("当前时刻:%.2f",t1.toSec());
ROS_INFO("当前时刻:%.2f",t2.toSec());
// [INFO] [1761726830.062000291]: 当前时刻:20.23
// [INFO] [1761726830.062025869]: 当前时刻:20.60
```

#### 持续时间

```cpp
ROS_INFO("吃到睡针了");
ros::Duration du(4);
du.sleep();
ROS_INFO("睡了4s");
```

#### 时间运算

```cpp
ros::Time begin = ros::Time::now();
ros::Duration du1(2);
// time和duration运算
ros::Time end = begin + du1;
ROS_INFO("begin:%.2f", begin.toSec());
ROS_INFO("end:%.2f", end.toSec());
// duration之间也可以加减运算
// time和time之间不能加，只能相减生成duration
```

#### 定时器

```cpp
Timer createTimer(Duration period, const TimerCallback& callback, bool oneshot = false, bool autostart = true) const;
```

```cpp
\param period 时间间隔
\param callback 回调函数
\param oneshot 如果设置为 true,只执行一次回调函数，设置为 false,就循环执行。
\param autostart 如果为true，返回已经启动的定时器,设置为 false，需要手动启动。
```

具体使用：

```cpp
ROS_INFO("-------------定时器--------------");
ros::Timer timer = nh.createTimer(ros::Duration(1), cb);
ros::spin();
```

还需要一个回调函数：

```cpp
void cb(const ros::TimerEvent& event)
{
    ROS_INFO("----------------");
}
```

