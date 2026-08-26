---
title: '基于树莓派的 DIY NAS'
createTime: 2026/07/23 20:08:53
permalink: /notes/Projects/raspberry-pi/diy-nas.html
---

# 基于树莓派的 DIY NAS

参考项目：https://the-diy-life.com/building-a-4-bay-3-5-nas-with-a-raspberry-pi-5-and-3d-printed-enclosure/

开源3D打印壳：https://makerworld.com/zh/models/1605027-raspberry-pi-5-based-4-bay-nas#profileId-1697964

参考资料1：https://docs.radxa.com/en/accessories/storage/penta-sata-hat

参考资料2：https://zhuanlan.zhihu.com/p/6740006083

参考资料3：https://tvtv.fun/omv7/3rd.html

## 实物图



## 配件清单及花费

| 序号 | 项目                   | 金额（元）  | 备注                             |
| ---- | ---------------------- | ----------- | -------------------------------- |
| 1    | 树莓派本体及相关配件   | 850.00      | 闲鱼二手收的                     |
| 2    | Penta SATA HAT         | 230.00      | 闲鱼全新                         |
| 3    | 铠侠二手 SSD           | 100.00      | 收的朋友的                       |
| 4    | 3D 打印外壳            | 141.50      | 找闲鱼哥们定制了颜色，打印了很久 |
| 5    | 小风扇                 | 3.50        | 买错接口，浪费了                 |
| 6    | DC 延长线              | 2.90        |                                  |
| 7    | 热熔螺母               | 1.30        |                                  |
| 8    | 各种螺丝螺母           | 7.16      | m3和m2.5的都有 |
| 9    | 12V 6A 电源            | 21.80       |                               |
| 10   | 树莓派主动散热器       | 10.87       | 自带的和hat不兼容，我重买了一个  |
| 11   | 新风扇 * 2             | 19.80    | 大4pin接口（母），后来被我搞废了一个，又买了一个 |
| 12   | SATA 延长线四根       | 13.36 |                                  |
| 13   | 绿联硬盘盒             | 22.61       |                                  |
| 14   | 3.5寸西数紫盘 * 4     | 0           | 白嫖的全新，可惜不是红盘              |
| 15   | 大4pin延长线 | 0.8        |               |
|      | 总计****               | **1425.6** ||

1400出头，还行，主要白嫖了硬盘没花钱，里面的所有配件都是参考的[该开源项目博客](https://the-diy-life.com/building-a-4-bay-3-5-nas-with-a-raspberry-pi-5-and-3d-printed-enclosure/)。

其中只有一个风扇和它不一样，原文是用的**小3pin母口的风扇**加一个**小3pin公口转大4pin母口**的转接线连到SATA HAT上。而我花了大量精力在国内各个平台找了好久，完全找不到**小3pin公口转大4pin母口**的转接线，最后偶然发现根本不需要转接线，直接买个大4pin母口的风扇就行。

后来发现我买的大4pin母口（其实是公母一体）体积太大了，顶到风扇，安装不进去，所以想把母口改造一下，剪掉一部分，结果被我剪坏了，又买了一根。之后又买了一根大4pin的延长线来接风扇的母口，这样不会顶到风扇。

## 组装过程

### 大部分的配件图

![c5a7100b7a86fb003731a7baa68986b3](https://cdn.jsdelivr.net/gh/salt235/tuchuang/img/c5a7100b7a86fb003731a7baa68986b3.png)

![90e75bda603aa4b621322f9e351f91cc](https://cdn.jsdelivr.net/gh/salt235/tuchuang/img/90e75bda603aa4b621322f9e351f91cc.png)

### 安装热熔螺母

还好家里有电烙铁。

![56d444c937591b95343b6bf1d7818713](https://cdn.jsdelivr.net/gh/salt235/tuchuang/img/56d444c937591b95343b6bf1d7818713.png)

### 安装硬盘到硬盘盒

![718c943b497fff8b7c6cf9aeb754dd0d](https://cdn.jsdelivr.net/gh/salt235/tuchuang/img/718c943b497fff8b7c6cf9aeb754dd0d.png)

### 安装风扇

![98526ee8d36e41eb05489250d9ef2251](https://cdn.jsdelivr.net/gh/salt235/tuchuang/img/98526ee8d36e41eb05489250d9ef2251.png)

![d78f0e3d5741b3c05c068102a16350fb](https://cdn.jsdelivr.net/gh/salt235/tuchuang/img/d78f0e3d5741b3c05c068102a16350fb.png)

这边发现这个公母一体的接口太大，顶到风扇了，后来想把接口的上面一部分剪掉一些，但是被我剪坏了，无奈重买了一个风扇。另外买了一个大4pin的延长线，避免顶到风扇。

![eae6aaf6ff2409454276ad3518b70616](https://cdn.jsdelivr.net/gh/salt235/tuchuang/img/eae6aaf6ff2409454276ad3518b70616.png)

很极限，最终这个延长线的方案成功了，后来也测试没问题，风扇正常转。

### 安装树莓派、HAT和sata线

![6868f649f0740d0445598d114a27a4fb](https://cdn.jsdelivr.net/gh/salt235/tuchuang/img/6868f649f0740d0445598d114a27a4fb.png)

### 安装前壳

![14d2f418f82be79eedecc5327cd22879](https://cdn.jsdelivr.net/gh/salt235/tuchuang/img/14d2f418f82be79eedecc5327cd22879.png)

### 插电测试

![6338f4ee703154a1c66ae37ff297e1d0](https://cdn.jsdelivr.net/gh/salt235/tuchuang/img/6338f4ee703154a1c66ae37ff297e1d0.png)

没问题，待机温度在41℃左右，还是非常优秀的。

### 完全体

后来把4块硬盘全装上了，解锁完全体，大功告成。

![dbac2884ed543a08ea23aa40e9671e93](https://cdn.jsdelivr.net/gh/salt235/tuchuang/img/dbac2884ed543a08ea23aa40e9671e93.png)

![18c9abe4ce69ac9856f626607ee795db](https://cdn.jsdelivr.net/gh/salt235/tuchuang/img/18c9abe4ce69ac9856f626607ee795db.png)

## 基础配置

### 禁用开机的电流检查

用上12V 72W的电源后，树莓派是通过SATA HAT的40Pin供电的，所以和树莓派的官方5V 27W的电源有区别。我的树莓派从 USB 硬盘启动后，检测到的整机供电能力只有 **3000 mA**，因此每次开机都停下来要求按钮确认。

编辑 `/boot/firmware/config.txt`，把 `usb_max_current_enable=1` 添加到文件末尾，保存后重启。

之后从 USB 硬盘启动时，就不会再停在这个确认界面了。该参数会重新允许 USB 启动，并把树莓派 5 的 USB 总电流限制从约 **600 mA 提高到 1600 mA**。

### 开启PCIe

编辑 `/boot/firmware/config.txt`，把 `dtparam=pciex1` 添加到文件末尾，保存后重启。

### 查看硬盘情况

```bash
lsblk
```
我目前是这样的结构：sdd - sde 是四个3.5寸HDD，通过SATA HAT利用PCIe连接；另外sda是一个2.5寸的SSD，通过usb连接树莓派作为一个系统盘。

```bash
yyx@YYX-Pi:~ $ lsblk
NAME   MAJ:MIN RM   SIZE RO TYPE MOUNTPOINTS
loop0    7:0    0     2G  0 loop
sda      8:0    0 223.6G  0 disk
├─sda1   8:1    0   512M  0 part /boot/firmware
└─sda2   8:2    0 223.1G  0 part /
sdb      8:16   0   1.8T  0 disk
sdc      8:32   0   1.8T  0 disk
sdd      8:48   0   1.8T  0 disk
sde      8:64   0   1.8T  0 disk
zram0  252:0    0     2G  0 disk [SWAP]
```

## OMV的基础配置

### 报错

```bash
wget https://github.com/OpenMediaVault-Plugin-Developers/installScript/raw/master/install
chmod +x install
sudo ./install -n
```

但是报错了，提示我：

```bash
[2026-08-17 14:50:15+0800] [omvinstall] log location :: /root/omv_install.log
This system is running a desktop environment!
Please use a Lite version of the image or
do not choose to install a desktop environment.
This install is not supported.
Search the forum for more info - https://forum.openmediavault.org
Exiting...
=== END 2026-08-17T14:50:15+08:00 rc=101 ===
```

无法安装在桌面版的ubuntu，靠ai通过一系列操作把桌面卸载了，切换为命令行系统，还是没用，所以我决定重装系统为 Raspberry Pi OS Lite 64-bit。

### 重新安装 Raspberry Pi OS Lite 64-bit 系统

重新安装好系统，经过了一系列的初始化操作，终于可以安装OMV了。

```bash
# 没有配置代理，直接用了加速前缀下载
wget -O install.sh \
"https://gh-proxy.org/https://raw.githubusercontent.com/openmediavault/openmediavault/master/install.sh"
# 测试 OMV 软件源是否可以访问
wget -O /dev/null https://packages.openmediavault.io/archive.key
# 安装
sudo sh install.sh
# 查看ip
hostname -I
```

得到ip是`192.168.3.202`，在win中打开`http://192.168.3.202`（需在局域网下），如果不在局域网，配置好tailscale之后，可以直接打开`http://yyx-pi`，成功打开面板，输入默认的账号密码。

![image-20260817163700033](https://cdn.jsdelivr.net/gh/salt235/tuchuang/img/image-20260817163700033.png)

### OMV 的初始配置

参考文档：https://tvtv.fun/omv7/3rd.html

- 修改密码

- 设置固定IP地址（但是我不在一个内网，用的tailscale，所以没必要设置）

- 设置时间和时区，把时间服务器换成了`time.windows.com`

- 修改自动退出时间

  默认情况下，OMV WebUI 会在 5 分钟没有操作后自动退出登陆。这虽然可以保护系统安全，但过短的时间往往需要频繁输入账号密码，操作体验不好。在`工作台-设置`里面把自动注销设置为1天。

- 设置通知邮件服务器

  必须现在qq邮箱网站里面开启 POP3/IMAP/SMTP 服务，生成 SMTP 的16位授权码，然后按如下方式填写，其中的密码就是刚刚生成的授权码。
  
  ![image-20260819140226127](https://cdn.jsdelivr.net/gh/salt235/tuchuang/img/image-20260819140226127.png)

## NAS 配置

### 格式化磁盘

在`存储器-磁盘`中，选中我的四个数据盘，点击左上角的`擦除`，选择`快速擦除`。

### RAID设置

> RAID 是 Redundant Array of Independent Disks 的缩写，中文名是独立磁盘冗余阵列。它是一种通过将多个硬盘组合起来，形成一个逻辑上的硬盘阵列，从而提高数据的读写速度和数据的冗余性的技术。

OMV 将 RAID 功能拆分成了单独的插件，所以需要先安装 `openmediavault-md` 插件，然后才能在管理界面中看到 RAID 的相关设置。在`系统-插件`中直接搜索并安装。

然后在`存储器-RAID管理`中，选择`raid5`并添加四个盘。

![image-20260819144847043](https://cdn.jsdelivr.net/gh/salt235/tuchuang/img/image-20260819144847043.png)

第一次 RAID5 初始化需要等待同步，可以用以下命令来查看进度。

```bash
cat /proc/mdstat
```

我这边花了四个多小时。

### 创建文件系统

> 文件系统是操作系统用来管理文件的一种机制，它决定了文件在硬盘上的的存储方式、文件的命名规则、文件的读写方式等。不同的操作系统使用不同的文件系统，比如 Windows 使用 NTFS、FAT32，Linux 使用 ext4、XFS 等。

在`存储器-文件系统`中，点击加号，给组好raid的整体大盘创建并挂载`ext4`文件系统。

### 创建共享文件夹

> 共享文件夹就是在文件系统上创建的一个个用来存放文件的普通目录。比如，你要存照片就创建一个叫 `照片` 的共享文件夹，要存电影就创建一个叫 `电影` 的共享文件夹。

先在`用户`里面把当前的yyx用户设置一个密码，便于后续访问NAS。

然后在 `存储器-共享文件夹`里面创建一个data目录，相当于“根目录”了，后续只要把这个文件夹共享，里面的所有子目录就等于共享了。

![image-20260819201458600](https://cdn.jsdelivr.net/gh/salt235/tuchuang/img/image-20260819201458600.png)

### 开启SMB

在`服务-SMB/CIFS-设置`里面，勾选最上面的`已启动`，和下面的`启用回收站`，其他保持不变。

然后在`服务-SMB/CIFS-共享`里面，勾选`已启动`，选择`data`目录，启用`回收站`,设置为`30天`。

### 测试

第一次测试发现速度只有8-14MB/s，很诧异，后来发现接的那个有线网口只有100兆，怪不得。而树莓派其实是千兆网口，根本没跑满。

```bash	
yyx@YYX-Pi:~ $ ethtool eth0
Settings for eth0:
        Supported ports: [ TP    MII ]
        Supported link modes:   10baseT/Half 10baseT/Full
                                100baseT/Half 100baseT/Full
                                1000baseT/Half 1000baseT/Full
        Supported pause frame use: Transmit-only
        Supports auto-negotiation: Yes
        Supported FEC modes: Not reported
        Advertised link modes:  10baseT/Half 10baseT/Full
                                100baseT/Half 100baseT/Full
                                1000baseT/Half 1000baseT/Full
        Advertised pause frame use: Transmit-only
        Advertised auto-negotiation: Yes
        Advertised FEC modes: Not reported
        Link partner advertised link modes:  10baseT/Half 10baseT/Full
                                             100baseT/Half 100baseT/Full
        Link partner advertised pause frame use: Symmetric Receive-only
        Link partner advertised auto-negotiation: Yes
        Link partner advertised FEC modes: Not reported
        Speed: 100Mb/s
        Duplex: Full
        Auto-negotiation: on
        master-slave cfg: preferred slave
        master-slave status: slave
        Port: Twisted Pair
        PHYAD: 1
        Transceiver: external
        MDI-X: Unknown
netlink error: Operation not permitted
        Link detected: yes
```
