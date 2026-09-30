---
title: "PyTorch Module 与容器"
createTime: 2026/09/27 00:00:00
permalink: /notes/MachineLearning/pytorch/module-containers.html
---

# PyTorch Module 与容器
在深度学习模型里面，有一些网络层需要放在一起使用，如 conv + bn + relu 的组合。Module的容器是将一组操作捆绑在一起的工具，在 pytorch 官方文档中把 Module 也定义为 Containers，或许是因为“Modules can also contain other Modules”。
卷积、池化和归一化等可组合的层见 [常用网络层](/notes/MachineLearning/pytorch/common-layers.html)；模型参数的保存和加载见 [Module APIs](/notes/MachineLearning/pytorch/module-apis.html)。
## forward
Module 是所有神经网络的基类，所有的模型都必须继承于 Module 类，并且它可以嵌套，一个Module 里可以包含另外一个 Module。
forward 之于 Module 等价于 getitem 之于 Dataset。forward 函数是模型每次调用的具体实现，所有的模型必须实现 forward 函数，否则调用时会报错。
```python
class CNN(nn.Module):
    def __init__(self):
        super().__init__()
        self.conv = nn.Conv2d(3, 16, kernel_size=3)
        self.relu = nn.ReLU()
        self.pool = nn.MaxPool2d(2)
    def forward(self, x):
        x = self.conv(x)
        x = self.relu(x)
        x = self.pool(x)
        return x
```
- __init__()：定义网络有什么
- forward() ：定义数据怎么走
## Sequential
它的作用是将一系列网络层按**固定的先后顺序**串起来，当成一个整体，调用时数据从第一个层**按顺序执行**到最后一个层。
sequential可以直接传module，也可以传OrderedDict，OrderedDict可以让容器里的每个module都有名字，方便调用。
举例：
```python
model = nn.Sequential(
          nn.Conv2d(1,20,5),
          nn.ReLU(),
          nn.Conv2d(20,64,5),
          nn.ReLU()
        )
# Using Sequential with OrderedDict. This is functionally the
# same as the above code
model = nn.Sequential(OrderedDict([
          ('conv1', nn.Conv2d(1,20,5)),
          ('relu1', nn.ReLU()),
          ('conv2', nn.Conv2d(20,64,5)),
          ('relu2', nn.ReLU())
        ]))
```
## ModuleList
ModuleList 是将各个网络层放到一个“列表”中，便于迭代的形式调用。它看起来很像 Python 的 list，但区别很关键：放进 ModuleList 里的网络层，会被 PyTorch 正确注册成模型的一部分。
```python
class MyModel(nn.Module):
    def __init__(self):
        super().__init__()
        self.layers = nn.ModuleList([
            nn.Linear(10, 20),
            nn.Linear(20, 30),
            nn.Linear(30, 2)
        ])
    def forward(self, x):
        for layer in self.layers:
            x = layer(x)
        return x
```
## ModuleDict
ModuleList 可以像 python 的 List 一样管理各个 module，但对于索引而言有一些不方便，因为它没有名字，需要记住是第几个元素才能定位到指定的层，这在深度神经网络中有一点不方便。
而 ModuleDict 就是可以像 python 的 Dict 一样为每个层赋予名字，可以根据网络层的名字进行选择性的调用网络层。
```python
    class MyModule2(nn.Module):
        def __init__(self):
            super(MyModule2, self).__init__()
            self.choices = nn.ModuleDict({
                    'conv': nn.Conv2d(3, 16, 5),
                    'pool': nn.MaxPool2d(3)
            })
            self.activations = nn.ModuleDict({
                    'lrelu': nn.LeakyReLU(),
                    'prelu': nn.PReLU()
            })
        def forward(self, x, choice, act):
            x = self.choices[choice](x)
            x = self.activations[act](x)
            return x
```
forward 里面参数的 choice 和 act 就是上面的名字：`'conv'`, `'prelu'`等等。
