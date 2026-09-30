---
title: "PyTorch 学习笔记"
createTime: 2026/09/30 00:00:00
permalink: /notes/MachineLearning/pytorch/
---

# PyTorch 学习笔记

这里汇总 PyTorch 的 Tensor、自动求导、数据管线、模型组织与训练评估基础。

## 快速入口

- [Tensor](/notes/MachineLearning/pytorch/tensor.html)：创建、索引、变形、广播和随机种子。
- [Dataset](/notes/MachineLearning/pytorch/dataset.html)：自定义数据集及文件夹、CSV 两种标注组织方式。
- [Module 与容器](/notes/MachineLearning/pytorch/module-containers.html)：`forward`、Sequential、ModuleList 和 ModuleDict。
- [损失函数](/notes/MachineLearning/pytorch/loss-functions.html)：L1Loss、CrossEntropyLoss 及其参数。
- [优化器](/notes/MachineLearning/pytorch/optimizers.html)：参数组、SGD 和学习率调整器。

## Tensor 与自动求导

### 学习路径

1. [创建和操作 Tensor](/notes/MachineLearning/pytorch/tensor.html)
2. [理解计算图与叶子结点](/notes/MachineLearning/pytorch/computation-graph.html)
3. [使用 autograd 与 backward](/notes/MachineLearning/pytorch/autograd.html)

## 数据管线

### 学习路径

1. [处理图像变换](/notes/MachineLearning/pytorch/transforms.html)
2. [实现 Dataset](/notes/MachineLearning/pytorch/dataset.html)
3. [按 batch 加载数据](/notes/MachineLearning/pytorch/dataloader.html)

## 模型构建与管理

- [常用网络层](/notes/MachineLearning/pytorch/common-layers.html)：卷积、池化、归一化、Dropout 和激活函数。
- [Module 容器](/notes/MachineLearning/pytorch/module-containers.html)：按顺序、列表或字典组织子模块。
- [Module APIs](/notes/MachineLearning/pytorch/module-apis.html)：设备迁移、参数保存加载与子模块查询。
- [模型保存、断点续训与微调](/notes/MachineLearning/pytorch/training-tips.html)：模型保存/加载、checkpoint 断点续训和微调。

## 训练与评估

### 学习路径

1. [根据任务选择损失函数](/notes/MachineLearning/pytorch/loss-functions.html)
2. [计算梯度并更新参数](/notes/MachineLearning/pytorch/optimizers.html)
3. [用分类指标评估模型](/notes/MachineLearning/pytorch/evaluation-metrics.html)

## 相关笔记

- [机器学习与深度学习](/notes/MachineLearning/)：机器学习、CNN、Attention 与 Transformer 基础。
