---
title: "PyTorch 损失函数"
createTime: 2026/09/29 00:00:00
permalink: /notes/MachineLearning/pytorch/loss-functions.html
---

# PyTorch 损失函数
损失函数（loss function）是用来**衡量模型输出与真实标签之间的差异**。
损失反向传播后得到的梯度，会由 [优化器](/notes/MachineLearning/pytorch/optimizers.html) 执行参数更新；分类结果的评估可参考 [模型评价指标](/notes/MachineLearning/pytorch/evaluation-metrics.html)。
针对不同的任务有不同的损失函数，例如回归任务常用 MSE(Mean Square Error)，分类任务常用 CE（Cross Entropy），这是根据标签的特征来决定的。而不同的任务还可以对基础损失函数进行各式各样的改进，如 Focal Loss 针对困难样本的设计，GIoU 新增相交尺度的衡量方式，DIoU 新增重叠面积与中心点距离衡量等等。
pytorch 设计了 21 种损失函数：
```python
nn.L1Loss                  # 平均绝对误差 MAE，计算预测值与真实值之间绝对差，常用于回归任务
nn.MSELoss                 # 均方误差 MSE，计算预测值与真实值之间平方差，常用于回归任务
nn.CrossEntropyLoss        # 交叉熵损失，最常用的多分类损失函数，内部包含 LogSoftmax + NLLLoss
nn.CTCLoss                 # CTC 损失，用于输入输出无法直接对齐的序列任务，如语音识别、OCR
nn.NLLLoss                 # 负对数似然损失，通常接在 LogSoftmax 后用于多分类任务
nn.PoissonNLLLoss          # 泊松负对数似然损失，用于服从泊松分布的计数型数据预测
nn.GaussianNLLLoss         # 高斯负对数似然损失，用于预测服从高斯分布的数据及其不确定性
nn.KLDivLoss               # KL 散度损失，衡量两个概率分布之间的差异，常用于知识蒸馏、VAE
nn.BCELoss                 # 二元交叉熵损失，用于二分类或多标签分类，输入通常已经经过 Sigmoid
nn.BCEWithLogitsLoss       # BCE + Sigmoid 的组合，用于二分类或多标签分类，数值稳定性更好，通常优先使用
nn.MarginRankingLoss       # 排序损失，让一个样本的预测分数比另一个样本高出一定 margin
nn.HingeEmbeddingLoss      # 铰链嵌入损失，用于判断两个样本应该接近还是远离，常用于相似性学习
nn.MultiLabelMarginLoss    # 多标签间隔损失，让正确标签的分数高于错误标签一定 margin
nn.HuberLoss               # Huber 损失，结合 MSE 和 MAE，对异常值比 MSE 更不敏感，常用于回归
nn.SmoothL1Loss            # 平滑 L1 损失，类似 Huber Loss，常用于目标检测中的边界框回归
nn.SoftMarginLoss          # 软间隔损失，用于二分类，通过平滑方式鼓励正负样本被正确分类
nn.MultiLabelSoftMarginLoss # 多标签软间隔损失，用于一个样本可以同时属于多个类别的多标签分类
nn.CosineEmbeddingLoss     # 余弦嵌入损失，根据余弦相似度让两个向量更加相似或更加不同
nn.MultiMarginLoss         # 多分类间隔损失，要求正确类别分数比其他类别至少高出一定 margin
nn.TripletMarginLoss       # 三元组损失，让 Anchor 更接近 Positive，同时远离 Negative
nn.TripletMarginWithDistanceLoss # 可自定义距离函数的三元组损失，是 TripletMarginLoss 的更灵活版本
```
## L1Loss
```python
_class_ torch.nn.L1Loss(_size_average=None_, _reduce=None_, _reduction='mean'_)
```
- ~~size_average (bool, optional) – 已舍弃使用的变量，功能已经由 reduction 代替实现，仍旧保留是为了旧版本代码可以正常运行。~~
- ~~reduce (bool, optional) – 已舍弃使用的变量，功能已经由 reduction 代替实现，仍旧保留是为了旧版本代码可以正常运行。~~
- reduction (string, optional) – 是否需要对 loss 进行“降维”，这里的 reduction 指是否将 loss 值进行取平均（mean）、求和（sum）或是保持原尺寸（none），这一变量在 pytorch 绝大多数损失函数中都有在使用，需要重点理解。
```python
    output = torch.ones(2, 2, requires_grad=True) * 0.5
    target = torch.ones(2, 2)
    params = "none mean sum".split()
	# 分别尝试 none, mean, sum 三种方式
    for p in params:
        loss_func = nn.L1Loss(reduction=p)
        loss_tmp = loss_func(output, target)
        print("reduction={}:loss={}, shape:{}".format(p, loss_tmp, loss_tmp.shape))
    print("\n")
```
输出结果：
```python
reduction=none:loss=tensor([[0.5000, 0.5000],
                            [0.5000, 0.5000]]), shape=torch.Size([2, 2])
reduction=mean:loss=0.5, shape=torch.Size([])
reduction=sum:loss=2.0, shape=torch.Size([])
```
none 方法结果是 tensor，其他两个方法是标量。
## CrossEntropyLoss
CrossEntropyLoss 用来衡量“模型对正确类别的预测有多差”，常用于单标签多分类任务。
```python
_class_ torch.nn.CrossEntropyLoss(_weight=None_, _size_average=None_, _ignore_index=-100_, _reduce=None_, _reduction='mean'_, _label_smoothing=0.0_)
```
- weight (Tensor, optional) – 类别权重，用于调整各类别的损失重要程度，常用于类别不均衡的情况。 If given, has to be a Tensor of size C
- ignore_index (int, optional) – 忽略某些类别不进行 loss 计算。
- ~~size_average (bool, optional) – 已舍弃使用的变量，功能已经由 reduction 代替实现，仍旧保留是为了旧版本代码可以正常运行。~~
- ~~reduce (bool, optional) – 已舍弃使用的变量，功能已经由 reduction 代替实现，仍旧保留是为了旧版本代码可以正常运行。~~
- reduction (string, optional) – 是否需要对 loss 进行“降维”，这里的 reduction 指是否将 loss 值进行取平均（mean）、求和（sum）或是保持原尺寸（none）。
- label_smoothing (float, optional) – 标签平滑参数，一个用于减少方差，防止过拟合的技巧。详细请看论文《 Rethinking the Inception Architecture for Computer Vision》。通常设置为0.01-0.1之间。
流程：
```
logits           # 原始分数
   ↓
LogSoftmax       # 把原始分数转换成“对数概率”
   ↓
NLLLoss          # 取真实类别对应的对数概率，再取负数
   ↓
loss
```
### CrossEntropyLoss 计算公式
先对 logits 做 Softmax：
$$
p_i=\frac{e^{z_i}}{\sum_{j=1}^{C}e^{z_j}}
$$
再与 one-hot 标签计算负对数损失：
$$
L=-\sum_{i=1}^{C}y_i\log(p_i)
$$
其中：
- $z_i$：第 $i$ 类的 logits
- $p_i$：Softmax 后第 $i$ 类的概率
- $y_i$：one-hot 标签
因为 one-hot 中只有真实类别的位置为 1，所以可简化为：
$$
L=-\log(p_{\text{true}})
$$
例子：
```python
import torch
import torch.nn as nn
import numpy as np
import math
params = "none mean sum".split()
output = torch.ones(2, 3, requires_grad=True) * 0.5
target = torch.from_numpy(np.array([0, 1])).type(torch.LongTensor)
for p in params:
loss_func = nn.CrossEntropyLoss(reduction=p)
loss_tmp = loss_func(output, target)
print("reduction={}:loss={}, shape:{}".format(p, loss_tmp, loss_tmp.shape))
# ----------------------- weight ------------------------------------
weight = torch.from_numpy(np.array([0.6, 0.2, 0.2])).float()
loss_f = nn.CrossEntropyLoss(weight=weight, reduction="none")
output = torch.ones(2, 3, requires_grad=True) * 0.5 # 假设一个三分类任务，batchsize为2个，假设每个神经元输出都为0.5
target = torch.from_numpy(np.array([0, 1])).type(torch.LongTensor)
loss = loss_f(output, target)
print('\n\nCrossEntropy loss: weight')
print('loss: ', loss) #
print('原始loss值为1.0986, 第一个样本是第0类，weight=0.6,所以输出为1.0986*0.6 =', 1.0986 * 0.6)
# ----------------------- ignore_index ------------------------------------
loss_f_1 = nn.CrossEntropyLoss(weight=None, reduction="none", ignore_index=1)
loss_f_2 = nn.CrossEntropyLoss(weight=None, reduction="none", ignore_index=2)
output = torch.ones(3, 3, requires_grad=True) * 0.5 # 假设一个三分类任务，batchsize为3个，假设每个神经元输出都为0.5
target = torch.from_numpy(np.array([0, 1, 2])).type(torch.LongTensor)
loss_1 = loss_f_1(output, target)
loss_2 = loss_f_2(output, target)
```
输出：
```python
reduction=none:loss=tensor([1.0986, 1.0986], grad_fn=<NllLossBackward0>), shape:torch.Size([2])
reduction=mean:loss=1.0986123085021973, shape:torch.Size([])
reduction=sum:loss=2.1972246170043945, shape:torch.Size([])
CrossEntropy loss: weight
loss:  tensor([0.6592, 0.2197], grad_fn=<NllLossBackward0>)
原始loss值为1.0986, 第一个样本是第0类，weight=0.6,所以输出为1.0986*0.6 = 0.65916
CrossEntropy loss: ignore_index
ignore_index = 1:  tensor([1.0986, 0.0000, 1.0986], grad_fn=<NllLossBackward0>)
ignore_index = 2:  tensor([1.0986, 1.0986, 0.0000], grad_fn=<NllLossBackward0>)
```
