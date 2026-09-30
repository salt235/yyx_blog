---
title: "PyTorch Tensor"
createTime: 2026/09/15 00:00:00
permalink: /notes/MachineLearning/pytorch/tensor.html
---

# PyTorch Tensor
张量是现代机器学习的基础。它的核心是一个数据容器，多数情况下，它包含数字，有时候它也包含字符串，但这种情况比较少。因此可以把它想象成一个数字的水桶。
Tensor 的 `requires_grad`、`grad_fn` 和 `is_leaf` 属性会在 [计算图](/notes/MachineLearning/pytorch/computation-graph.html) 与 [自动求导](/notes/MachineLearning/pytorch/autograd.html) 中继续使用。
这里有一些存储在各种类型张量的公用数据集类型：
- **3维 = 时间序列**
- **4维 = 图像**
- **5维 = 视频**
为什么图像要4维？在机器学习工作中，我们经常要处理不止一张图片或一篇文档——我们要处理一个集合。我们可能有10,000张郁金香的图片，这意味着，我们将用到4D张量：
```
(batch_size, width, height, channel) = 4D
```
## Tensor 属性
Tensor主要有以下八个**主要属性**，data，dtype，shape，device，grad，grad_fn，is_leaf，requires_grad。
- data：多维数组，最核心的属性，其他属性都是为其服务的;
- dtype：多维数组的数据类型，tensor数据类型如下，常用到的三种已经用红框标注出来；
- shape：多维数组的形状;
- device: tensor所在的设备，cpu或cuda;
- grad，grad_fn，is_leaf和requires_grad就与Variable一样，都是梯度计算中所用到的。
## Tensor 创建
### 1. 随机 Tensor
```python
x = torch.rand(4, 3)
```
创建 `4×3`、元素范围为 `[0, 1)` 的随机 Tensor。
```python
x = torch.randn_like(x, dtype=torch.float)
```
创建与 `x` 形状相同、服从标准正态分布的随机 Tensor。
### 2. 全 0 / 全 1 Tensor
```python
x = torch.zeros(4, 3, dtype=torch.long)
```
创建 `4×3` 的全 0 Tensor，类型为 `long`。
```python
x = x.new_ones(4, 3, dtype=torch.double)
```
基于 `x` 创建全 1 Tensor，类型为 `double`。
### 3. 根据数据创建
```python
x = torch.tensor([5.5, 3])
```
直接根据已有数据创建 Tensor。
### 4. 查看形状
```python
x.size()
x.shape
```
两者都可以查看 Tensor 的形状：
```text
torch.Size([4, 3])
```
### 5. 常用方法
```python
torch.rand()        # 0~1 随机数
torch.randn_like()  # 同形状，正态随机数
torch.zeros()       # 全 0
torch.tensor()      # 根据已有数据创建
x.new_ones()        # 基于 x 创建全 1 Tensor
x.shape             # 查看形状
x.size()            # 查看形状
```
## Tensor 基本运算
### 1. Tensor 加法
```python
x = torch.rand(4, 3)
y = torch.rand(4, 3)
```
几种加法写法效果相同：
```python
x + y
torch.add(x, y)
y.add(x)
```
注意：
```python
y.add(x)
```
不会修改 `y` 本身，只会返回计算结果。
### 2. 原地操作
带 `_` 的操作会直接修改 Tensor 本身：
```python
y.add_(x)
```
等价于：
```python
y += x
```
此时 `y` 的数据会发生变化。
常见规律：
```python
add()    # 不修改原 Tensor
add_()   # 修改原 Tensor
```
PyTorch 中很多以 `_` 结尾的方法都表示 **原地操作（in-place operation）**。
### Tensor 索引与切片
```python
x = torch.rand(4, 3)
```
取第 2 列：
```python
x[:, 1]
```
其中：
```text
:   表示所有行
1   表示第 2 列（下标从 0 开始）
```
取第 1 行：
```python
x[0, :]
```
也可以直接写：
```python
x[0]
```
### 3. 切片可能共享数据
```python
y = x[0, :]
y += 1
```
此时：
```python
print(y)
print(x[0, :])
```
会发现 `x` 的第一行也发生了变化。
原因是：
> `y` 并不是完全独立的新 Tensor，而是和 `x` 共享底层数据。
如果希望复制一份独立数据：
```python
y = x[0, :].clone()
```
### 4. 改变 Tensor 形状
```python
x = torch.randn(4, 4)
```
原始形状：
```text
4 × 4
```
展开成一维：
```python
y = x.view(16)
```
结果：
```text
torch.Size([16])
```
重新变成 `2 × 8`：
```python
z = x.view(-1, 8)
```
结果：
```text
torch.Size([2, 8])
```
`-1` 表示让 PyTorch 自动计算这一维：
```python
x.view(-1, 8)
```
总共 16 个元素，因此自动推导为：
```text
2 × 8
```
### 5. view() 通常共享数据
```python
x = torch.randn(4, 4)
y = x.view(16)
```
此时 `x` 和 `y` 通常共享底层数据。
例如：
```python
x += 1
```
再查看：
```python
print(y)
```
会发现 `y` 中的数据也发生了变化。
因此：
> `view()` 主要改变 Tensor 的形状，一般不会复制数据。
### 6. Tensor 转 Python 标量
如果 Tensor 中只有一个元素：
```python
x = torch.randn(1)
```
此时：
```python
type(x)
```
结果：
```text
torch.Tensor
```
使用：
```python
x.item()
```
可以取出其中的 Python 数值：
```python
type(x.item())
```
结果：
```text
float
```
因此：
```python
x.item()
```
常用于将单元素 Tensor 转换成普通 Python 标量。
### 6. 常用操作速记
```python
x + y              # Tensor 加法
torch.add(x, y)     # Tensor 加法
x.add(y)            # 加法，不修改 x
x.add_(y)           # 原地加法，修改 x
x[:, 1]             # 取第 2 列
x[0, :]             # 取第 1 行
x.clone()            # 复制 Tensor
x.view(16)           # 改变形状为一维
x.view(-1, 8)        # 自动计算其中一个维度
x.size()             # 查看形状
x.shape              # 查看形状
x.item()             # 单元素 Tensor → Python 标量
```
## 广播机制
当对两个形状不同的 Tensor 按元素运算时，可能会触发广播(broadcasting)机制：先适当复制元素使这两个 Tensor 形状相同后再按元素运算。
```python
x = torch.arange(1, 3).view(1, 2)
print(x)
y = torch.arange(1, 4).view(3, 1)
print(y)
print(x + y)
```
结果为：
```bash
tensor([[1, 2]])
tensor([[1],
        [2],
        [3]])
tensor([[2, 3],
        [3, 4],
        [4, 5]])
```
## Tensor 的随机种子
随机种子（random seed）是编程语言中基础的概念，大多数编程语言都有随机种子的概念，它主要用于实验的复现。针对随机种子 pytorch 也有一些设置函数。
| [`seed`](https://pytorch.org/docs/stable/generated/torch.seed.html#torch.seed)                            | 获取一个随机的随机种子。                                                     |
| --------------------------------------------------------------------------------------------------------- | ---------------------------------------------------------------- |
| [`manual_seed`](https://pytorch.org/docs/stable/generated/torch.manual_seed.html#torch.manual_seed)       | 手动设置随机种子，建议设置为42，这是近期一个玄学研究。说42有效的提高模型精度。当然大家可以设置为你喜欢的，只要保持一致即可。 |
| [`initial_seed`](https://pytorch.org/docs/stable/generated/torch.initial_seed.html#torch.initial_seed)    | 返回初始种子。                                                          |
| [`get_rng_state`](https://pytorch.org/docs/stable/generated/torch.get_rng_state.html#torch.get_rng_state) | 获取随机数生成器状态。                                                      |
| [`set_rng_state`](https://pytorch.org/docs/stable/generated/torch.set_rng_state.html#torch.set_rng_state) | 设定随机数生成器状态。这两怎么用暂时未知。                                            |
以上均是设置 cpu 上的张量随机种子，在 cuda 上是另外一套随机种子，如 torch.cuda.manual_seed_all(seed)， 这些到 cuda 模块再进行介绍，这里只需要知道cpu和cuda上需要分别设置随机种子。
