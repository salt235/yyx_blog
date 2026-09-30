---
title: "PyTorch 图像变换 transforms"
createTime: 2026/09/26 00:00:00
permalink: /notes/MachineLearning/pytorch/transforms.html
---

# PyTorch 图像变换 transforms
transforms 是广泛使用的图像变换库，包含二十多种基础方法以及多种组合功能，通常可以用 Compose 把各方法串联在一起使用。大多数的transforms类都有对应的 functional transforms ，可供用户自定义调整。transforms 提供的主要是 PIL 格式和 Tensor 的变换，并且对于图像的通道也做了规定，默认情况下一个 batch 的数据是 (B, C, H, W)  形状的张量。
图像变换通常在 [Dataset](/notes/MachineLearning/pytorch/dataset.html) 中调用，再由 [DataLoader](/notes/MachineLearning/pytorch/dataloader.html) 组装成 batch。
## Compose
此类用于包装一系列的transforms方法，在其内部会通过for循环依次调用各个方法。
```python
transforms_func = transforms.Compose([
    transforms.Resize((2, 2)),
    transforms.ToTensor()
])
```
## Resize
```python
Resize(size, interpolation=, max_size=None, antialias=None)
```
支持对PIL或Tensor对象的缩放。关于size的设置有些讲究，请有 int 方式与 tuple 方式。int方式是会根据长宽比等比例的缩放图像。
```python
# tuple 方式
transforms_func = transforms.Compose([
    transforms.Resize((2, 2)),
    transforms.ToTensor()
])
# int 方式
transforms_func = transforms.Compose([
    transforms.Resize(5),
    transforms.ToTensor()
])
```
- tuple 方式一般没什么问题，直接指定了输出的尺寸。
- int 方式则是保持长宽比，然后把短边缩放到 5，另一边等比例缩放，所以可能会导致不同图片尺寸不一样，会报错。有一种方法：AlexNet 论文中提到先等比例缩放再裁剪出 224\*224 的正方形区域。
## ToTensor
将 PIL 对象或 nd.array 对象转换成 tensor，并且对数值缩放到 [0, 1] 之间，并且对通道进行右移。
## Normalize
```python
Normalize(mean, std, inplace=False)
```
对tensor对象进行逐通道的标准化，具体操作为减均值再除以标准差，一般使用 imagenet 的128万数据R\G\B三通道统计得到的 mean 和 std ，mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225]。
## FiveCrop 和 TenCrop
具体使用方式是一张图片经过多区域裁剪得到5/10张图片，同时放到模型进行推理，得到5/10个概率向量，然后取它们的平均/最大/最小得到这一张图片的概率。
FiveCrop 表示对图片进行上下左右以及中心裁剪，获得 5 张图片，并返回一个**list**，这导致我们需要额外处理它们，使得他们符合其它 transforms 方法的形式——3D-tensor。
```python
# 会报错的版本
transforms_func = transforms.Compose([
    transforms.Resize((256, 256)),
    transforms.FiveCrop(224), # 从图像中裁剪五块 224 × 224 区域
    transforms.ToTensor(), # 报错
    transforms.Normalize([0.4], [0.2])
])
```
关键点：FiveCrop 返回的不是一张图，而是包含五张 PIL 图像的 **tuple**，所以执行 ToTensor 会报错：`TypeError: pic should be PIL Image or ndarray. Got <class 'tuple'>`。
正确写法：
```python
transforms_func = transforms.Compose([
    transforms.Resize((10, 10)),
    transforms.FiveCrop(8),
    transforms.Lambda(
        lambda crops: torch.stack([ToTensor()(crop) for crop in crops])
    ),
    transforms.Normalize([0.4], [0.2])
])
```
TenCrop 同理，在 FiveCrop 的基础上增加水平镜像，获得 10 张图片，并返回一个 list。
