---
title: "PyTorch 优化器"
createTime: 2026/09/29 00:00:00
permalink: /notes/MachineLearning/pytorch/optimizers.html
---

# PyTorch 优化器
优化器使用 [自动求导](/notes/MachineLearning/pytorch/autograd.html) 得到的梯度，根据 [损失函数](/notes/MachineLearning/pytorch/loss-functions.html) 的结果更新模型参数。
## Optimizer
Optimizer 类是所有具体优化器的基类。
### 基础属性参数组 param_groups
例如在 finetune 过程中，通常让前面层的网络采用较小的学习率，后面几层全连接层采用较大的学习率，
这是我们就要把网络的参数划分为两组，每一组有它对应的学习率。正是因为这种针对不同参数需要不同的更新策略的需求，才有了参数组的概念。
参数组是一个 list，其元素是一个 dict，dict 中包含，所管理的参数，对应的超参，例如学习率，momentum，weight_decay等等。
```python
    w1 = torch.randn(2, 2)
    w1.requires_grad = True
    w2 = torch.randn(2, 2)
    w2.requires_grad = True
    w3 = torch.randn(2, 2)
    w3.requires_grad = True
    # 一个参数组
    optimizer_1 = optim.SGD([w1, w3], lr=0.1)
    print('len(optimizer.param_groups): ', len(optimizer_1.param_groups))
    print(optimizer_1.param_groups, '\n')
    # 两个参数组
    optimizer_2 = optim.SGD([{'params': w1, 'lr': 0.1},
                             {'params': w2, 'lr': 0.001}])
    print('len(optimizer.param_groups): ', len(optimizer_2.param_groups))
    print(optimizer_2.param_groups)
```
### 基础方法
- zero_grad()
功能：清零所管理参数的梯度。由于pytorch**不会自动清零梯度**，因此需要在 optimizer 中手动清零，然后再执行反向传播，得出当前iteration的loss对权值的梯度。
- step()
功能：执行一步更新，依据当前的梯度进行更新参数
- add_param_group(param_group)
功能：给 optimizer 管理的参数组中增加一组参数，可为该组参数**定制 lr, momentum, weight_decay** 等，在 finetune 中常用。
例如：`optimizer_1.add_param_group({'params': w3, 'lr': 0.001, 'momentum': 0.8})`
- state_dict()
功能：获取当前 state 属性。
通常在保存模型时同时保存优化器状态，用于断点保存，下次继续从当前状态训练；
- load_state_dict(state_dict)
功能：加载所保存的 state 属性，恢复训练状态。
对优化器工作方式熟悉后，再看 Optimizer 的属性和方法就简单了，下面通过一个具体的优化算法来熟悉完整的优化器使用。
## SGD
SGD(stochastic gradient descent，随机梯度下降)是深度学习模型优化过程中最基础、最受欢迎、最稳定的一个，即使优化算法层出不穷，仅 pytorch 就提供了十三个，但目前绝大多数论文中仍旧采用 SGD 进行训练，因此 SGD 必须掌握。
SGD核心理论知识是梯度下降 (gradient descent)，即沿着梯度的负方向，是变化最快的方向。而随机则指的是一次更新中，采用了一部分样本进行计算，即一个 batch 的数据可以看作是整个训练样本的随机采样。
$$ \theta_{t+1}=\theta_t-\eta \nabla_\theta L(\theta_t)$$
其中：
```
θ      ：模型参数，比如权重 w、偏置 b
η      ：学习率 learning rate
∇L     ：loss 对参数的梯度
```
核心训练流程：
```python
outputs = model(data)          # 1. 前向传播
optimizer.zero_grad()          # 2. 清空梯度
loss = loss_f(outputs, labels) # 3. 计算 loss
loss.backward()                # 4. 反向传播，计算梯度
optimizer.step()               # 5. SGD 更新参数
```
此外补充，两个优化器的参数:
```python
momentum=0.9      # 动量：参考之前的更新方向，加快收敛、减少震荡
weight_decay=5e-4 # 权重衰减：限制参数过大，起到正则化、防过拟合作用
```
## 关于学习率调整器
- `StepLR`：每隔固定 epoch，lr 乘一个系数。
- `CosineAnnealingLR`：lr 平滑下降。
- `ReduceLROnPlateau`：验证 loss 不再改善时降 lr。
