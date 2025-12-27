---
title: ROS常用组件
createTime: 2025/12/27 14:44:18
permalink: /notes/ROS/roi7c4bh/
---
## ROS常用组件

### 坐标的msg消息

#### 1.geometry_msgs/TransformStamped

```
std_msgs/Header header                     #头信息
  uint32 seq                                #|-- 序列号
  time stamp                                #|-- 时间戳
  string frame_id                            #|-- 坐标 ID
string child_frame_id                    #子坐标系的 id
geometry_msgs/Transform transform        #坐标信息
  geometry_msgs/Vector3 translation        #偏移量
    float64 x                                #|-- X 方向的偏移量
    float64 y                                #|-- Y 方向的偏移量
    float64 z                                #|-- Z 方向上的偏移量
  geometry_msgs/Quaternion rotation        #四元数
    float64 x                                
    float64 y                                
    float64 z                                
    float64 w
```

四元数用于表示坐标的相对姿态

#### 2.geometry_msgs/PointStamped

```
std_msgs/Header header                    #头
  uint32 seq                                #|-- 序号
  time stamp                                #|-- 时间戳
  string frame_id                            #|-- 所属坐标系的 id
geometry_msgs/Point point                #点坐标
  float64 x                                    #|-- x y z 坐标
  float64 y
  float64 z
```

### 静态坐标的转换

#### 发布方

需要导入的包有老三样和 tf2、tf2_ros、tf2_geometry_msgs、roscpp rospy std_msgs geometry_msgs。

```cpp
#include "ros/ros.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "geometry_msgs/TransformStamped.h"
#include "tf2/LinearMath/Quaternion.h"

/*
    需求：发布两个坐标系的相对关系
*/

int main(int argc, char *argv[])
{
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "static_pub");
    ros::NodeHandle nh;
    
    // 发布对象和之前不一样
    tf2_ros::StaticTransformBroadcaster pub;
    
    // 发布的消息
    geometry_msgs::TransformStamped tfs;
    tfs.header.stamp = ros::Time::now(); // 时间戳获取
    tfs.header.frame_id = "base_link"; // 父亲坐标系，小车本体
    tfs.child_frame_id = "laser"; // 雷达坐标系
    // 设置坐标系的偏移量
    tfs.transform.translation.x = 0.2;
    tfs.transform.translation.y = 0.0;
    tfs.transform.translation.z = 0.5;
    
    // 四元数设置：将 欧拉角数据 转换成 四元数
    tf2::Quaternion qtn;
    qtn.setRPY(0,0,0); // 欧拉角的单位是弧度,这里是偏航俯仰啥的

    tfs.transform.rotation.w = qtn.getW();
    tfs.transform.rotation.x = qtn.getX();
    tfs.transform.rotation.y = qtn.getY();
    tfs.transform.rotation.z = qtn.getZ();

    pub.sendTransform(tfs);
    ros::spin();
    
    return 0;
}
```

#### 订阅方

```cpp
#include "ros/ros.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "geometry_msgs/PointStamped.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h" //注意: 调用 transform 必须包含该头文

/*
    需求：订阅方接受刚刚发布的坐标相对关系。
            传入一个坐标点，用tf实现转换。
*/

int main(int argc, char *argv[])
{
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "static_sub");
    ros::NodeHandle nh;

    // buffer用来缓存接受的数据
    tf2_ros::Buffer buffer;
    tf2_ros::TransformListener listener(buffer);

    // 组织一个坐标点数据
    geometry_msgs::PointStamped ps;
    ps.header.frame_id = "laser"; // 
    ps.header.stamp = ros::Time::now();
    ps.point.x = 2.0;
    ps.point.y = 3.0;
    ps.point.z = 5.0;

    // 转换算法
    ros::Rate rate(10);
    while(ros::ok())
    {
        try
        {
            // 核心实现，将ps转换为相对base_link的坐标点
            geometry_msgs::PointStamped ps_out;
            // 调用的buffer的转换函数
            ps_out = buffer.transform(ps, "base_link"); // 参数一：被转换的坐标点；参数二：参考的坐标系
            // 输出
            ROS_INFO("转换后的坐标值：(%.2f,%.2f,%.2f)",ps_out.point.x, 
            ps_out.point.y, ps_out.point.z);           
        }
        catch(const std::exception& e)
        {
            ROS_INFO("异常");
        }
        
        rate.sleep();
        ros::spinOnce();
    }

    return 0;
}
```

这边try_catch可以防止异常，因为可能sub没接收到pub的base_link信息，就会抛异常。

#### 补充1

当坐标系之间的相对位置固定时，那么所需参数也是固定的: 父系坐标名称、子级坐标系名称、x偏移量、y偏移量、z偏移量、x 翻滚角度、y俯仰角度、z偏航角度，实现逻辑相同，参数不同，那么 ROS 系统就已经封装好了专门的节点，使用方式如下:

```
rosrun tf2_ros static_transform_publisher x偏移量 y偏移量 z偏移量 z偏航角度 y俯仰角度 x翻滚角度 父级坐标系 子级坐标系
```

示例:`rosrun tf2_ros static_transform_publisher 0.2 0 0.5 0 0 0 /baselink /laser`

也建议使用该种方式直接实现静态坐标系相对信息发布。

#### 补充2

可以借助于rviz显示坐标系关系，具体操作:

- 新建窗口输入命令:rviz;
- 在启动的 rviz 中设置Fixed Frame 为 base_link;
- 点击左下的 add 按钮，在弹出的窗口中选择 TF 组件，即可显示坐标关系。

### 动态坐标的转换

#### 发布方

```cpp
#include "ros/ros.h"
#include "turtlesim/Pose.h" 
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/TransformStamped.h"
#include "tf2/LinearMath/Quaternion.h"

/*
    需求：订阅乌龟的位姿消息，转换为相对于窗体的坐标关系,并发布
    话题：/turtle1/pose
    消息：turtlesim/Pose
            float32 x
            float32 y
            float32 theta
            float32 linear_velocity
            float32 angular_velocity
*/

void doPose(const turtlesim::Pose::ConstPtr& pose)
{
    // 获取位姿信息，转换为坐标系的相对关系
    // 1.创建发布对象(static 避免重复创建)
    static tf2_ros::TransformBroadcaster pub;
    // 2.组织被发布的数据
    geometry_msgs::TransformStamped ts;
    ts.header.frame_id = "world";
    ts.header.stamp = ros::Time::now();
    ts.child_frame_id = "turtle1";
    // 坐标系偏移量
    ts.transform.translation.x = pose->x;
    ts.transform.translation.y = pose->y;
    ts.transform.translation.z = 0;

    // 坐标系四元数
    // 位姿消息没有四元数，只有偏航角度，又已知乌龟是2D的，没有翻滚和俯仰角度
    tf2::Quaternion qtn;
    qtn.setRPY(0, 0, pose->theta);
    ts.transform.rotation.x = qtn.getX();
    ts.transform.rotation.y = qtn.getY();
    ts.transform.rotation.z = qtn.getZ();
    ts.transform.rotation.w = qtn.getW();

    // 3.发布
    pub.sendTransform(ts);
}

int main(int argc, char *argv[])
{
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "dynamic_pub");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("turtle1/pose",100, doPose);
    ros::spin();
    return 0;
}
```

#### 订阅方

```cpp
#include "ros/ros.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "geometry_msgs/PointStamped.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h" 

int main(int argc, char *argv[])
{
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "dynamic_sub");
    ros::NodeHandle nh;

    tf2_ros::Buffer buffer;
    tf2_ros::TransformListener listener(buffer);

    ros::Rate r(1);
    while(ros::ok())
    {
        // 生成一个坐标点（相对于子坐标系）
        geometry_msgs::PointStamped ps;
        ps.header.frame_id = "turtle1";
        // 时间戳不是now了
        ps.header.stamp = ros::Time(0.0);// 注意
        ps.point.x = 1;
        ps.point.y = 1;
        ps.point.z = 0;
        geometry_msgs::PointStamped ps_out;
        try
        {
            ps_out = buffer.transform(ps, "world");
            // 输出
            ROS_INFO("转换后的坐标值：%.2f, %.2f,%.2f,参考的坐标系是：%s",
            ps_out.point.x,
            ps_out.point.y,
            ps_out.point.z,
            ps_out.header.frame_id.c_str()
            );
        }
        catch(const std::exception& e)
        {
            std::cerr << e.what() << '\n';
        }

        r.sleep();
        ros::spinOnce();
    }

    return 0;
}
```

### 多坐标变换

#### 发布方

这里直接用静态坐标变换来发布

```
<launch>
    <node pkg = "tf2_ros" type = "static_transform_publisher" name = "son1" args = "5 0 0 0 0 0 /world /son1" output = "screen" />
    <node pkg = "tf2_ros" type = "static_transform_publisher" name = "son2" args = "3 0 0 0 0 0 /world /son2" output = "screen" />
</launch>
```

#### 订阅方

```cpp
#include "ros/ros.h"
#include "tf2_ros/transform_listener.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "geometry_msgs/TransformStamped.h"
#include "geometry_msgs/PointStamped.h"

/* 
    需求：计算son1和son2的相对关系
         计算son1中的某个坐标点在son2中的坐标值
*/

int main(int argc, char *argv[])
{
    setlocale(LC_ALL,"");
    ros::init(argc,argv,"sub_frames");
    ros::NodeHandle nh;
    tf2_ros::Buffer buffer; 
    tf2_ros::TransformListener listener(buffer);
    ros::Rate r(1);

    // 创建son1中的坐标点
    geometry_msgs::PointStamped psSon1;
    psSon1.header.stamp = ros::Time::now();
    psSon1.header.frame_id = "son1";
    psSon1.point.x = 1.0;
    psSon1.point.y = 2.0;
    psSon1.point.z = 3.0;

    while(ros::ok())
    {
        try
        {
            // 计算son1相对于son2的坐标关系
                // lookupTranform返回坐标的相对关系，参数一是父亲坐标系，参数二是儿子坐标系
            geometry_msgs::TransformStamped tfs = buffer.lookupTransform("son2", "son1", ros::Time(0.0));
            ROS_INFO("son1相对于son2的消息:父亲：%s, 儿子:%s, 偏移量:(%.2f, %.2f, %.2f)",
                    tfs.header.frame_id.c_str(),
                    tfs.child_frame_id.c_str(),
                    tfs.transform.translation.x,
                    tfs.transform.translation.y,
                    tfs.transform.translation.z
                    );


            //计算son1中的某个坐标点在son2中的坐标值
            geometry_msgs::PointStamped psSon2 = buffer.transform(psSon1, "son2");
            ROS_INFO("坐标点在son2中的值:(%.2f. %.2f, %.2f),参考的坐标系为%s",
                    psSon2.point.x,
                    psSon2.point.y,
                    psSon2.point.z,
                    psSon2.header.frame_id.c_str()
                    );
        }
        catch(const std::exception& e)
        {
            std::cerr << e.what() << '\n';
        }
        
    }

    return 0;
}
```

### 坐标系关系查看

自动生成pdf

```bash
sudo apt install ros-noetic-tf2-tools
```

``` bash
rosrun tf2_tools view_frames.py
```

