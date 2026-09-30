---
title: "PyTorch 自动求导"
createTime: 2026/09/15 00:00:00
permalink: /notes/MachineLearning/pytorch/autograd.html
---

# PyTorch 自动求导
PyTorch 中，所有神经网络的核心是 `autograd` 包。autograd包为张量上的所有操作提供了自动求导机制。它是一个在运行时定义 ( define-by-run ）的框架，这意味着反向传播是根据代码如何运行来决定的，并且每次迭代可以是不同的。
本节以前置的 [计算图](/notes/MachineLearning/pytorch/computation-graph.html) 为基础，并会用到 [Tensor](/notes/MachineLearning/pytorch/tensor.html) 的梯度相关属性。
## Autograd
### 1. 简介
`autograd`：PyTorch 的自动求导机制，用于计算 Loss 对参数的梯度。
```python
import torch
x = torch.tensor(3.0, requires_grad=True)
y = x ** 2
y.backward()
print(x.grad)   # tensor(6.)
```
$$
y=x^2,\quad \frac{dy}{dx}=2x=6
$$
### 2. `requires_grad=True`
表示 PyTorch 需要追踪该 Tensor 后续参与的运算。
```python
x = torch.tensor(3.0, requires_grad=True)
y = x ** 2
z = y * 2
```
```text
x → x² → y → ×2 → z
```
默认 `requires_grad=False`。
### 3. 计算图
PyTorch 在前向计算时自动记录 Tensor 之间的运算关系。
```python
x = torch.tensor(2.0, requires_grad=True)
y = x ** 2
z = y * 3
out = z + 1
```
```text
x → x² → y → ×3 → z → +1 → out
```
$$
out=3x^2+1
$$
```python
out.backward()
print(x.grad)   # tensor(12.)
```
### 4. `grad_fn`
`grad_fn`：记录 Tensor 是通过什么运算产生的。
```python
x = torch.tensor(2.0, requires_grad=True)
y = x ** 2
z = y * 3
print(x.grad_fn)   # None
print(y.grad_fn)   # PowBackward0
print(z.grad_fn)   # MulBackward0
```
- 手动创建的 Tensor：`grad_fn=None`
- 运算产生的 Tensor：有对应的 `grad_fn`
### 5. 叶子张量 Leaf Tensor
用户直接创建、位于计算图起点的 Tensor。
```python
x = torch.tensor(2.0, requires_grad=True)
y = x ** 2
z = y * 3
```
```text
x → y → z
↑
叶子张量
```
神经网络中的 `weight`、`bias` 通常都是叶子张量。
反向传播后，梯度默认保存在叶子张量的 `.grad` 中。
### 6. `backward()`
`.backward()`：从当前 Tensor 开始，沿计算图反向传播，通过链式法则计算梯度。
```python
x = torch.tensor(2.0, requires_grad=True)
y = x ** 3
y.backward()
print(x.grad)   # tensor(12.)
```
$$
y=x^3,\quad \frac{dy}{dx}=3x^2=12
$$
### 7. `backward()` 的参数
当 `y` 是标量时：
```python
y.backward()
```
等价于：
```python
y.backward(torch.tensor(1.0))
```
默认从上游梯度 `1` 开始反向传播。
当 `y` 是向量或矩阵时，需要传入一个与 `y` 同形的 Tensor：
```python
x = torch.tensor([1., 2., 3.], requires_grad=True)
y = x ** 2
y.backward(torch.tensor([1., 1., 1.]))
print(x.grad)   # tensor([2., 4., 6.])
```
传入的 Tensor 表示每个输出元素对应的**上游梯度/权重**。
```text
标量 y：
backward() 默认梯度 = 1
非标量 y：
backward(v)
v 与 y 同形，用于指定各输出如何参与反向传播
```
### 8. 核心流程
```text
requires_grad=True
        ↓
前向计算建立计算图
        ↓
grad_fn 记录运算
        ↓
backward()
        ↓
链式法则反向求导
        ↓
梯度保存到 .grad
```
## 梯度
梯度表示 **Loss 对模型参数变化的敏感程度**，用于决定参数更新方向。
```python
loss.backward()
````
`backward()` 会根据计算图和链式法则自动计算梯度，结果保存在参数的 `.grad` 中。
```python
print(parameter.grad)
```
梯度默认会累加，因此每轮训练前通常需要清零：
```python
optimizer.zero_grad()
loss.backward()
optimizer.step()
```
训练核心流程：
```text
前向传播 → 计算 Loss
         ↓
     loss.backward()
         ↓
       计算梯度
         ↓
    optimizer.step()
         ↓
       更新参数
```
补充：
- 标量 Loss：直接 `loss.backward()`
- 非标量 Tensor：`backward(v)` 需要传入同形 Tensor
- Jacobian / 向量-Jacobian 积：知道它们本质是链式法则即可，现阶段无需深入
- 推理时不需要梯度，可使用 `torch.no_grad()`
