---
title: ML_Note
createTime: 2025/12/27 14:44:18
permalink: /notes/Lee_DL/eezwc65m/
---
# 李宏毅ML_Note

## 机器学习是什么？

> **机器学习就是让机器能自动找到一个函数（function）**。

### 有哪些函数？

- 回归（Regression）
  输出是一个连续的数值、标量，比如PM2.5预测。
- 分类（Classification）
  输出是一个离散的值。
  - 二分类（Binary Classification）的输出就是0或1、Yes或No、…，比如文本情感分析的输出可以是正面和负面。
  - 多分类（Multi-Category Classification）的输出就是[1,2,3,…,N]，比如图像分类里判断一张图片是猫还是狗还是杯子。
- 生成（Generation）
  很多教科书把机器学习划分为回归问题和分类问题，但其实不止这两种问题，还有其它问题，比如生成（Generation）。
  生成（Generation）指让机器学习如何创造/生成，比如生成文本、图片等。

### 机器学习在做什么？

- 函数拟合（你选什么函数？）

- 损失函数（怎么判断好坏？）

- 梯度下降（如何把函数调到更好？）

### Sigmoid函数

$y = c\frac{1}{1 + e^{-(b+wx_1)}}$

![image-20251204180137061](https://cdn.jsdelivr.net/gh/salt235/tuchuang@main/macImg/image-20251204180137061.png)

