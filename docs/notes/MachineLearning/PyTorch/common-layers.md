---
title: "PyTorch 常用网络层"
createTime: 2026/09/27 00:00:00
permalink: /notes/MachineLearning/pytorch/common-layers.html
---

# PyTorch 常用网络层
常用网络层的概念见：[CNN 与 ResNet](/notes/MachineLearning/528u1pz0/)。
## Convolutional Layers
```python
torch.nn.Conv2d(in_channels, out_channels, kernel_size, stride=1, padding=0, dilation=1, groups=1, bias=True, padding_mode='zeros')
```
主要参数：
- **in_channels** (int) – Number of channels in the input image。输入这个网络层的图像的通道数是多少。
- **out_channels** (int) – Number of channels produced by the convolution。此网络层输出的特征图的通道数是多少，等价于卷积核数量是多少。
- **kernel_size** (int or tuple) – Size of the convolving kernel。卷积核大小。
- **stride** (int or tuple, optional) – Stride of the convolution. Default: 1。卷积核卷积过程的步长。
- **padding** (int, tuple or str, optional) – Padding added to all four sides of the input. Default: 0。对于输入图像的四周进行填充的数量进行控制，可指定填充像素数量，也可以指定填充模式，如 `"same"`、`"valid"`。
- **padding_mode** (string, optional) – `'zeros'`, `'reflect'`, `'replicate'` or `'circular'`. Default: `'zeros'`。填充的像素值如何确定，默认填充 0。
- **dilation** (int or tuple, optional) – Spacing between kernel elements. Default: 1。孔洞卷积的孔洞大小。
- **groups** (int, optional) – Number of blocked connections from input channels to output channels. Default: 1。分组卷积的分组。
- **bias** (bool, optional) – If True, adds a learnable bias to the output. Default: True。是否采用偏置。
建议结合各种动图进行学习，推荐这个[repo](https://github.com/vdumoulin/conv_arithmetic)。
## Pooling Layers
作用是将特征图分辨率变小，通常减小一半。对池化层进行划分，分为最大值池化、平均值池化、分数阶池化、基于范数的池化。分别对应 torch.nn 中的 Maxpool, Avgpool, FractionalMaxPool, LPPool。下面是最大池化：
```python
torch.nn.MaxPool2d(kernel_size, stride=None, padding=0, dilation=1, return_indices=False, ceil_mode=False)
```
主要参数：
- kernel_size – 池化窗口大小
- stride – 滑窗步长
- padding – 原图填充大小
- dilation – 孔洞大小
- return_indices – 是否返回最大值所在位置，主要在 torch.nn.MaxUnpool2d 中使用，是上采样的一种策略
- ceil_mode – 当无法整除时，是向下取整还是向上取整，默认为向下取整。
针对最大池化还有一个特殊的地方是它可以记录最大值所在的位置，供上采样时（MaxUnpool2d）所用。
### Adaptive Pooling
上面针对池化像素如何取值进行划分，其实针对窗口大小的选择也可划分，还有另外一种特殊的池化方法，那就是 AdaptiveXpool， 它的作用是自适应窗口大小，保证经过池化层之后的图像尺寸是固定的，这个在接入全连接层之前经常会见到。
使用也很方便，只需要设置想要的输出大小即可，例如自适应的最大池化：
```python
torch.nn.AdaptiveMaxPool2d(output_size, return_indices=False)
```
## Padding Layers
改网络中常用到，功能是给特征图周围填充一定的像素，调整特征图分辨率的一种方法。既然是填充就涉及两个问题，填充多少个像素？像素应该如何确定？
针对第二个问题，可将 padding laye r划分为三类，镜像填充、边界重复填充，指定值填充、零值填充，分别对应nn的三大类，nn.ReflectionPad2d， nn.ReplicationPad2d， nn.ZeroPad2d， nn.ConstantPad2d。
- 镜像填充（Reflect Padding）**：用边界附近的像素进行**镜像反射**来填充，例如 1 2 3 → 3 2 | 1 2 3 | 2 1。
- **边界重复填充（Replicate Padding）**：直接用**最边缘的像素值重复**填充，例如 1 2 3 → 1 1 | 1 2 3 | 3 3。
- **指定值填充（Constant Padding）**：用**人为指定的固定值**填充边缘，例如指定 5，就用 5 填充。
- **零值填充（Zero Padding）**：指定值填充的特殊情况，边缘统一用 **0** 填充，也是卷积中最常见的 padding。
## Linear Layers
Linear Layers 包含4个层分别是 nn.Identity，nn.Linear， nn.Bilinear， nn.LazyLinear
- nn.Identity 是恒等映射，不对输入做任何变换，它通常用于占位。
- nn.Linear 就是大家熟悉的全连接层(Fully Connection Layer)，可实现 y= Wx + b
- nn.Bilinear 是双线性层，它有两个输入，实现公式 y = x1Wx2 +b
- nn.LazyLinear 是nn.Linear的lazy版本，也就是懒惰的Linear层，它在第一次推理时自动根据输入特征图的尺寸来设定in_features，免去了手动计算in_features的麻烦。
## Normalization Layers
包含主流的标准化网络层，分别有 BN、LN、IN、GN以及早期的LRN。这一些列的层已经成为现在深度学习模型的标配，它们充当一种正则，对数据的分布进行变换，使数据分布变到0均值，1标准差的形式。实验结果发现这样做可以加速模型训练，让模型更稳定，精度更高。
例如最出名的当属2015年提出的 BatchNorm：
```python
torch.nn.BatchNorm2d(num_features, eps=1e-05, momentum=0.1, affine=True, track_running_stats=True, device=None, dtype=None)
```
BatchNorm 会对输入进行减均值、除以标准差、乘以γ、加β的操作。
![image.png](https://cdn.jsdelivr.net/gh/salt235/tuchuang/macImg/20260928151843410.png)
## Dropout Layers
Dropout 的主要作用是：防止神经网络过拟合，提高泛化能力。训练时随机将部分神经元输出置 0，减少神经元之间的依赖，缓解过拟合；测试时关闭。
 Dropout使用注意事项：
- Dropout 通常用于全连接层附近，也可以用于卷积网络等结构。
- Dropout 训练时会随机将部分神经元输出置 0，但不会改变神经元数量或张量维度。
- 为保持期望尺度不变，PyTorch 会把保留下来的值按 `1/(1-p)` 放大。
### Alpha Dropout
AlphaDropout 是专门配合 **SELU 激活函数** 使用的 Dropout。
它和普通 Dropout 的区别是：
- 普通 Dropout：随机把部分值变成 `0`
- AlphaDropout：随机把部分值变成一个特定的负值，并做缩放
- 目的：尽量保持数据的 **均值和方差不变**
## Non-linear Layers
Non-linear Layer 就是**非线性层**，给神经网络加入非线性能力，让网络能够学习复杂关系，这样网络才能拟合复杂的曲线、边界和模式。在神经网络里通常指各种激活函数层：
```
nn.ReLU()
nn.Sigmoid()
nn.Tanh()
nn.LeakyReLU()
nn.ELU()
nn.GELU()
nn.SELU()
```
更通俗的划分是：
- 非 softmx 的，如sigmoid、tanh、ReLU、PReLU等
-  softmax 系列
对于softmax需要简单讲一讲，softmax的作用是将一个向量转换为一个概率分布的形式，以便于实现loss的计算，计算过程如下图所示：
![Softmax 计算示意图](https://cdn.jsdelivr.net/gh/salt235/tuchuang/macImg/20260928154535131.png)
看着一头雾水，其实很好理解。一个概率向量它的要求至少有这两个
1. 非负
2. 求和等于1
对于非负，用上幂函数，就可以实现了；
对于求和对于1，那就所有元素除以一个求和项，所有元素再加起来的时候分子就等于分母，自然求和等于1了，Softmax的设计思路真巧妙！
