---
title: "PyTorch 计算图"
createTime: 2026/09/25 00:00:00
permalink: /notes/MachineLearning/pytorch/computation-graph.html
---

# PyTorch 计算图
这一节是[自动求导](/notes/MachineLearning/pytorch/autograd.html)的前置知识。
计算图（Computational Graphs）是一种描述运算的“语言”，它由节点(Node)和边(Edge)构成。
~~**节点**表示数据，如标量，向量，矩阵，张量等~~
~~**边**表示运算，如加、减、乘、除、卷积、relu等；~~
记录所有节点和边的信息，可以方便地完成自动求导，假设有这么一个计算：
> y = (x+ w) * (w+1)
将每一步细化为：
> a = x + w
>
> b = w + 1
>
> y = a * b
得到计算图如下：
![image.png](https://cdn.jsdelivr.net/gh/salt235/tuchuang/macImg/20260925222238469.png)
## 计算图求导
![image.png](https://cdn.jsdelivr.net/gh/salt235/tuchuang/macImg/20260925222357548.png)
我们发现，所有的偏微分计算所需要用到的数据都是基于w和x的，这里，w和x就称为**叶子结点**。
## 叶子结点
张量有一个属性是 is_leaf, 就是用来指示一个张量是否为叶子结点的属性。
```python
import torch
w = torch.tensor([1.], requires_grad=True)
x = torch.tensor([2.], requires_grad=True)
a = torch.add(w, x)
b = torch.add(w, 1)     # retain_grad()
y = torch.mul(a, b)
y.backward()
print(w.grad)
# 查看叶子结点
print("is_leaf:\n", w.is_leaf, x.is_leaf, a.is_leaf, b.is_leaf, y.is_leaf)
# 查看梯度
print("gradient:\n", w.grad, x.grad, a.grad, b.grad, y.grad)
# 查看 grad_fn
print("grad_fn:\n", w.grad_fn, x.grad_fn, a.grad_fn, b.grad_fn, y.grad_fn)
```
结果为：
```bash
is_leaf:
 True True False False False
gradient:
 tensor([5.]) tensor([2.]) None None None
grad_fn:
 None None <AddBackward0 object at 0x106e04f40> <AddBackward0 object at 0x106e06e30> <MulBackward0 object at 0x106e05c60>
```
- 补充知识点1：**非叶子结点**在梯度反向传播结束后释放。只有叶子节点的梯度得到保留，中间变量的梯度默认不保留；在 pytorch 中，非叶子结点的梯度在反向传播结束之后就会被释放掉，如果需要保留的话可以对该结点设置 `retain_grad()`。
- 补充知识点2：**grad_fn** 是用来记录创建张量时所用到的运算，在链式求导法则中会使用到。
