---
title: RustDesk配置中继云服务器
createTime: 2026/04/14 13:33:00
tags:
    - 技术
    - 文档
---


## 1.给服务器安装代理

我使用的是腾讯云的轻量云服务器（2c2g），如何配置clash参考：https://github.com/nelvko/clash-for-linux-install。

直接执行：

```bash
git clone --branch master --depth 1 https://gh-proxy.org/https://github.com/nelvko/clash-for-linux-install.git \
  && cd clash-for-linux-install \
  && bash install.sh
```

按提示导入订阅后，先刷新环境变量，然后启动代理：

```bash
source ~/.bashrc
clashon
```

## 2. 服务器操作

参考教程：https://rustdesk.com/docs/en/self-host/。

开放端口:

```bash
# TCP 21114-21119
# UDP 21116
在腾讯云的防火墙页面中设置
```

在服务器中的安装脚本（开启代理）:

```bash
wget https://raw.githubusercontent.com/techahold/rustdeskinstall/master/install.sh
chmod +x install.sh
./install.sh
```

按照提示操作，最后会出现：

```bash
Your IP/DNS Address is xxx.xxx.xxx.xxx
Your public key is xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
```

## 3. 客户端操作

在RustDesk的客户端中，“设置”-“网络”。

ID服务器填服务器的ip地址，Key就填那个public key，另外两个留空。









