---
title: 一些关于PSDK开发的C++基础知识补全
createTime: 2026/06/25 13:14:14
permalink: /notes/LanguageLearning/cpp/psdk-basics.html
---

主要知识点：
```
1. std::queue 队列
2. std::mutex 锁
3. std::thread 线程
4. std::atomic 原子变量
5. JSON 格式
6. UDP 基本概念
7. Linux 命名管道 FIFO
8. 最后把它们串起来做一个 mock server
```

## 1. std::queue

主要掌握这几个函数就行：

```c++
push()   // 放入队列
front()  // 查看队首元素
pop()    // 弹出队首元素
empty()  // 判断是否为空
```

案例：

```c++
std::queue<std::string> cmd_queue;

cmd_queue.push("takeoff");
cmd_queue.push("land");

std::stirng cmd = cmd_queue.front(); // takeoff
cmd.queue.pop();
```

 

## 2. std::mutex

队列本身不难，但是如果多个线程与队列交互，那么要加锁才行。

主要记住以下这个案例：

```c++
std::mutex queue_mutex;
std::queue<std::string> cmd_queue;

void push_command(const std::string &cmd) {
  std::lock_guard<std::mutex> lock(queue_mutex);
  cmd_queue.push(cmd);
}
```

记住这个加锁的固定写法：

```c++
std::lock_guard<std_mutex> lock(queue_mutex);
```

**凡是多个线程共享一个变量，就要考虑加锁。**

## 3. std::thread

多线程，每个线程可以单独做事情，让程序同时做多件事。

一个简单的例子：

```c++
#include <iostream>
#include <thread>

void worker1() {
    std::cout << "线程1运行中\n";
}

void worker2() {
    std::cout << "线程2运行中\n";
}

void worker3() {
    std::cout << "线程3运行中\n";
}

int main() {
    std::thread t1(worker1); // 创建t1线程，调用worker1
    std::thread t2(worker2);
    std::thread t3(worker3);

    t1.join(); // 主线程等待子线程结束
    t2.join();
    t3.join();

    std::cout << "所有子线程都结束了，主线程继续执行\n";

    return 0;
}
```

注意，这不是说 `t1` 结束后 `t2` 才开始。**t1、t2、t3 在创建后就已经同时跑起来了。**
 `join()` 只是主线程在这里“等它们收尾”。

## 4. std::atomic

`atomic` 可以理解为“轻量级的线程安全变量”。

如果有一个全局变量`bool running = true;`，多个线程同时读写它，而且没有加锁，这就叫**数据竞争**，这是不安全不可靠的。因此需要一个能安全支持多线程读写的变量，也就是`atomic`。其实也可以用`mutex`加锁来控制一个变量，但是很麻烦。

一个完整的例子。

```c++
#include <iostream>
#include <thread>
#include <atomic>
#include <chrono>

std::atomic<bool> running(true);

void worker() {
    while (running) {
        std::cout << "子线程运行中...\n";
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    std::cout << "子线程退出\n";
}

int main() {
    std::thread t(worker);

    std::this_thread::sleep_for(std::chrono::seconds(5));

    running = false;

    t.join();

    std::cout << "主线程退出\n";
    return 0;
}
```

运行结果：

```c++
> /Users/yyx/Desktop/DJI开发/tmp
子线程运行中...
子线程运行中...
子线程运行中...
子线程运行中...
子线程运行中...
子线程退出
主线程退出
```

## 5. JSON

### 概念

 JSON本质就是一种文本格式，用来表达结构化数据。

常见头文件，起别名为json：

```c++
#include <iostream>
#include <string>
#include <nlohmann/json.hpp>

using json = nlohmann::json;
```

```c++
json j;
j["command"] = "takeoff";
j["command_id"] = 1;

ctd::cout << j << std::endl;
```

输出的结果为：`{"command":"takeoff","command_id":1}`

### 将JSON对象转成字符串。

```c++
json j;

j["command"] = "takeoff";
j["command_id"] = 1;

std::string msg = j.dump(); // dump可以将JSON对象转换成字符串，便于一些通信。

std::cout << msg << std::endl;
```

### 把字符串解析为JSON

假设收到一个字符串`std::string msg = R"({"command":"takeoff","command_id":1})";`，如何解析它呢？

前面的 `R"( ... )"` 是 C++ 的原始字符串写法，里面的双引号不用转义。

```c++
std::string msg = R"({"command":"takeoff","command_id":1})";
json j = json::parse(msg); // 解析
std::cout << j["command"] << std::endl;
std::cout << j["command_id"] << std::endl;
```

注意，直接输出 `j["command"]` 会带引号，因为它仍然是 JSON 类型，上面的输出：

```c++
"takeoff"
1
```

如果要转换为C++的`std::string`，用`.get<类型>()`：

```c++
std::string command = j["command"].get<std::string>();
int command_id = j["command_id"].get<int>();
```

### 遍历JSON

```c++
int main() {
    json j = {
        {"command", "position_control"},
        {"x", 1.0},
        {"y", 0.0},
        {"z", 0.5},
        {"yaw", 30.0}
    };

    for (auto& item : j.items()) {
        std::cout << "key = " << item.key()
                  << ", value = " << item.value()
                  << std::endl;
    }

    return 0;
}
```



### 如何判断字段是否存在

```c++
if (j.contains("command")){
  …………
}
```

### 给字段设置默认值

```c++
double yaw = j.value("yaw", 0.0);
```

### 异常处理

```c++
try {
    json j = json::parse(msg);

    if (!j.contains("command")) {
        std::cerr << "缺少 command 字段\n";
        return;
    }

    std::string command = j["command"].get<std::string>();

    std::cout << "收到命令：" << command << std::endl;

} catch (const json::parse_error& e) {
    std::cerr << "JSON 格式错误：" << e.what() << std::endl;
} catch (const json::type_error& e) {
    std::cerr << "JSON 字段类型错误：" << e.what() << std::endl;
}
```

这是两种常见的错误：

```c++
parse_error：字符串不是合法 JSON
type_error：字段类型和你想读的类型不匹配
```

## 6. UDP

UDP全称是User Datagram Protocol（用户数据报协议），是一种“**不建立连接，直接发消息”**的通信方式，而TCP是要事先建立连接的。在PSDK项目中，UDP就是程序之间用网络发消息的一种方式，直接发到对方端口，不用建立连接。

```
后端服务器  --发送 JSON 命令-->  PSDK 程序
PSDK 程序  --上报无人机状态-->  后端服务器
```

### IP和端口

```
IP 地址：发给哪台机器
端口 port：发给这台机器上的哪个程序
```

### socket

socket 是程序用来收发网络数据的“插座”或者“通信端口对象”。

UDP程序一般的步骤：

```
1. 创建 socket
2. 绑定本地端口 bind
3. 接收数据 recvfrom
4. 发送数据 sendto
5. 关闭 socket
```

接收方：

```
创建 UDP socket
绑定到本地端口，比如 9000
等待别人往这个端口发数据
收到后得到数据内容和发送者地址
```

发送方：

```
创建 UDP socket
指定目标 IP + 端口
sendto 发出去
```

### ACK

ACK就是确认消息，接收端收到发送端的消息之后，会给它返回确认消息。这就是在UDP之上，自己做一个可靠性补充。

后端发送：

```json
{
  "command": "takeoff",
  "command_id": 1001
}
```

可以返回给他：

```json	
{
  "type": "ack",
  "command_id": 1001,
  "status": "received"
}
```

### UDP的基本C++接收代码：

```c++
#include <iostream>
#include <cstring>
#include <unistd.h>
#include <arpa/inet.h>

int main(){

    // 1. 创建socket
    int sockfd = socket(AF_INET, SOCK_DGRAM, 0); // 创建socket，返回一个编号
    // AF_INET：使用 IPv4
    // SOCK_DGRAM：使用 UDP
	// 0：使用默认协议
    if (sockfd < 0) {
        std::cout << "socket创建失败" << std::endl;
        return 1;
    }

    // 2. 设置本地地址和端口
    sockaddr_in server_addr{}; // 创建一个sockaddr_in结构体，设置接收方的地址信息
    server_addr.sin_family = AF_INET; // IPv4
    server_addr.sin_addr.s_addr = INADDR_ANY; // 监听所有可用的网络接口
    server_addr.sin_port = htons(9000); // 端口号
    // sockaddr_in包含了sin_family、sin_port、sin_addr等字段，用于描述一个IPv4的地址和端口

    // 3. 绑定socket到本地地址和端口
        // sockfd：要绑定的socket描述符
        // (sockaddr*)&server_addr：指向sockaddr结构体的指针，
        // sizeof(server_addr)：sockaddr结构体的大小
    if (bind(sockfd, (sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
        std::cout << "bind失败" << std::endl;
        close(sockfd);
        return 1;
    }

    std::cout << "UDP服务器已启动，正在监听9000端口..." << std::endl;

    char buffer[1024]; // 用于接收数据的缓冲区

    while(1) {
        sockaddr_in client_addr{}; // 用于存储发送方的地址信息
        socklen_t client_len = sizeof(client_addr); // 发送方地址结构体的大小

        // 4. 接收数据
        ssize_t len = recvfrom(
            sockfd,
            buffer,
            sizeof(buffer) - 1, // 留出一个字节用于存储字符串结束
            0,
            (sockaddr*)&client_addr,
            &client_len
        );

        if (len < 0) {
            std::cout << "接收数据失败" << std::endl;
            continue; // 出错则继续接收
        }

        // 5. 手动添加字符串结束符
        buffer[len] = '\0'; // 将接收到的数据转换为字符串

        // 6. 打印发送方的IP地址和端口号
        char client_ip[INET_ADDRSTRLEN]; // 用于存储发送方的IP地址
        inet_ntop(AF_INET, &client_addr.sin_addr, client_ip, sizeof(client_ip)); // 将网络字节序的IP地址转换为点分十进制字符串
        int client_port = ntohs(client_addr.sin_port); // 将网络字节序的端口号转换为主机字节序

        std::cout << "接收到来自 " << client_ip << ":" << client_port << " 的数据: " << buffer << std::endl;

        // 7. 给发送方发送ACK
        const char* ack = "ACK"; // ACK消息

        sendto(
            sockfd,
            ack,
            strlen(ack),
            0,
            (sockaddr*)&client_addr,
            client_len
        );
    }

    close(sockfd); // 关闭socket
    return 0;
}
```

### UDP的基本C++发送代码

```c++
#include <iostream>
#include <cstring>
#include <unistd.h>
#include <arpa/inet.h>

int main() {
    // 1. 创建 UDP socket
    int sockfd = socket(AF_INET, SOCK_DGRAM, 0);

    if (sockfd < 0) {
        std::cerr << "socket 创建失败" << std::endl;
        return 1;
    }

    // 2. 设置接收方地址127.0.0.1:9000,这是本地机地址和端口号
    sockaddr_in receiver_addr{};
    receiver_addr.sin_family = AF_INET;
    receiver_addr.sin_port = htons(9000);

    // 将字符串形式的 IP 地址转换为网络字节序的二进制形式
    if (inet_pton(AF_INET, "127.0.0.1", &receiver_addr.sin_addr) <= 0) {
        std::cerr << "IP 地址转换失败" << std::endl;
        close(sockfd);
        return 1;
    }

    // 3. 要发送的消息
    const char* msg = R"({"command":"takeoff","command_id":1})";

    // 4. 发送 UDP 数据
    ssize_t sent_len = sendto(
        sockfd,
        msg,
        strlen(msg),
        0,
        (sockaddr*)&receiver_addr,
        sizeof(receiver_addr)
    );

    if (sent_len < 0) {
        std::cerr << "sendto 发送失败" << std::endl;
        close(sockfd);
        return 1;
    }

    std::cout << "已发送消息：" << msg << std::endl;

    // 5. 等待接收方返回 ACK
    char buffer[1024];

    sockaddr_in from_addr{};
    socklen_t from_len = sizeof(from_addr);

    ssize_t recv_len = recvfrom(
        sockfd,
        buffer,
        sizeof(buffer) - 1,
        0,
        (sockaddr*)&from_addr,
        &from_len
    );

    if (recv_len > 0) {
        buffer[recv_len] = '\0';
        std::cout << "收到回复：" << buffer << std::endl;
    } else {
        std::cerr << "没有收到回复" << std::endl;
    }

    close(sockfd);
    return 0;
}
```

## 7. 命名管道FIFO

UDP 是网络通信，FIFO（First In First Out 先进先出）是 Linux 本机进程之间通信。

FIFO表面上是一个文件，但是它不是普通文件。一个进程写进去的数据会被另一个进程读走。它主要用于通信，不是用于保存。

### 读取FIFO的基本C++代码

基本流程：

```
创建 FIFO
打开 FIFO 读端
循环读取数据
打印收到的内容
```

```c++
#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <sys/stat.h>
#include <cerrno>
#include <cstring>

int main() {
    const char * FIFO_PATH = "/tmp/test_fifo";

    // 1.创建FIFO
    if (mkfifo(FIFO_PATH, 0666) < 0) {
        if (errno == EEXIST) {
            std::cout << "FIFO 已经存在" << std::endl;
        } else {
            std::cout << "FIFO 创建失败: " << strerror(errno) << std::endl;
            return 1;
        }
    }

    std::cout << "准备打开FIFO读端:" << FIFO_PATH << std::endl;
    
    // 2.打开FIFO读端
    int fd = open(FIFO_PATH, O_RDONLY); // O_RDONLY表示只读模式
    if (fd < 0) {
        std::cout << "打开FIFO失败: " << strerror(errno) << std::endl;
        return 1;
    }

    std::cout << "FIFO已打开，等待数据..." << std::endl;

    char buffer [1024];

    while(1){
        // 3. 读取数据
        ssize_t len = read(fd, buffer, sizeof(buffer) - 1); // -1是为了给字符串末尾留出空间
        if (len > 0) {
            buffer[len] = '\0'; // 确保字符串以空字符结尾
            std::cout << "接收到数据: " << buffer << std::endl;
        }
        else if (len == 0) {
            std::cout << "写端关闭，FIFO已关闭" << std::endl;
            break; // 写端关闭，退出循环
        } else {
            std::cout << "读取数据失败: " << strerror(errno) << std::endl;
            break; // 读取失败，退出循环
        }
    }
    
    close(fd); // 关闭FIFO读端
    return 0;

}
```

### 接收FIFO的基本C++代码

```c++
#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <cstring>
#include <cerrno>

int main() {
    const char* FIFO_PATH = "/tmp/test_fifo";

    std::cout << "准备打开 FIFO 写端：" << FIFO_PATH << std::endl;

    // 1. 打开 FIFO 写端
    int fd = open(FIFO_PATH, O_WRONLY);

    if (fd < 0) {
        std::cerr << "open 写端失败：" << std::strerror(errno) << std::endl;
        return 1;
    }

    // 2. 写入数据
    const char* msg = R"({"command":"takeoff","command_id":1})";

    ssize_t len = write(fd, msg, strlen(msg));

    if (len < 0) {
        std::cerr << "write 失败：" << std::strerror(errno) << std::endl;
        close(fd);
        return 1;
    }

    std::cout << "已写入数据：" << msg << std::endl;

    close(fd);
    return 0;
}
```

记得要先运行接收端，来创建FIFO文件，然后再打开发送端，发送数据。

## 8. 综合小demo——模拟PSDK项目的一些通信和框架
[见小项目页面](https://github.com/salt235/mock_server)
