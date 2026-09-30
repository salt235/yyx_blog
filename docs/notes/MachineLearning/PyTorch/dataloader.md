---
title: "PyTorch DataLoader"
createTime: 2026/09/26 00:00:00
permalink: /notes/MachineLearning/pytorch/dataloader.html
---

# PyTorch DataLoader
DataLoader 提供了丰富的功能，下面介绍常用的功能，高阶功能等到具体项目中再进行分析。
DataLoader 以 [Dataset](/notes/MachineLearning/pytorch/dataset.html) 提供的单个样本为输入，并按 batch 组织训练数据。
- **dataset**：它是一个 Dataset 实例，要能实现从索引（indices/keys）到样本的映射。（即getitem函数）
- **batch_size**：每个 batch 的样本量
- **shuffle**：是否对打乱样本顺序。**训练集通常要打乱它！**验证集和测试集无所谓。
- **sampler**：设置采样策略。后面会详细展开。
- **batch_sampler**：设置采样策略， batch_sampler 与 sampler 二选一，具体选中规则后面代码会体现。
- **num_workers**： 设置多少个子进程进行数据加载（data loading）
- **collate_fn**：组装数据的规则， 决定如何将一批数据组装起来。
- **pin_memory**：是否使用锁页内存，具体行为是“the data loader will copy Tensors into CUDA pinned memory before returning them”
- **drop_last**：每个 epoch 是否放弃最后一批不足 batchsize 大小的数据，即无法被 batchsize 整除时，最后会有一小批数据，是否进行训练，如果数据量足够多，通常设置为True。这样使模型训练更为稳定，千万不要理解为某些数据被舍弃了，因为每个 epoch，dataloader 的采样都会重新 shuffle，因此不会存在某些数据被真正的丢弃。
## 示例
```python
# 前面省略了 dataset 部分，这里 train 集有5张图片
if __name__ == "__main__":
	root_dir = r"./data/datasets/mini-hymenoptera_data/train"
	normalize = transforms.Normalize([0.485, 0.456, 0.406], [0.229, 0.224, 0.225]) # 来自ImageNet数据集统计值
	transforms_train = transforms.Compose([
		transforms.Resize((224, 224)),
		transforms.ToTensor(),
		normalize
	])
	# 新建一个 dataset
	train_set = AntsBeesDataset(root_dir, transform=transforms_train) # 加入transform
	# 3种batch加载
	train_loader_bs2 = DataLoader(dataset=train_set, batch_size=2, shuffle=True)
	train_loader_bs3 = DataLoader(dataset=train_set, batch_size=3, shuffle=True)
	train_loader_bs2_drop = DataLoader(dataset=train_set, batch_size=2, shuffle=True, drop_last=True)
	# i: 当前第几个 batch
	# inputs: 当前 batch 的输入
	# target: 当前 batch 对应的标签
	for i, (inputs, target) in enumerate(train_loader_bs2):
		print(i, inputs.shape, target.shape, target)
	for i, (inputs, target) in enumerate(train_loader_bs3):
		print(i, inputs.shape, target.shape, target)
	for i, (inputs, target) in enumerate(train_loader_bs2_drop):
		print(i, inputs.shape, target.shape, target)
```
结果：
```bash
0 torch.Size([2, 3, 224, 224]) torch.Size([2]) tensor([0, 1])
1 torch.Size([2, 3, 224, 224]) torch.Size([2]) tensor([1, 0])
2 torch.Size([1, 3, 224, 224]) torch.Size([1]) tensor([0])
0 torch.Size([3, 3, 224, 224]) torch.Size([3]) tensor([1, 0, 1])
1 torch.Size([2, 3, 224, 224]) torch.Size([2]) tensor([0, 0])
0 torch.Size([2, 3, 224, 224]) torch.Size([2]) tensor([1, 0])
1 torch.Size([2, 3, 224, 224]) torch.Size([2]) tensor([0, 0])
```
## 分析
### 1. DataLoader 工作流程
```
Dataset
↓
根据索引调用 __getitem__()
↓
得到单个样本 (image, label)
↓
DataLoader 按 batch_size 取多个样本
↓
将多个样本组装成一个 batch
↓
返回 inputs, target
```
### 2. 遍历 DataLoader
```
for i, (inputs, target) in enumerate(train_loader):
```
其中：
```
i：当前第几个 batch，从 0 开始
inputs：当前 batch 的输入图片
target：当前 batch 对应的标签
```
例如：
```
0 torch.Size([2, 3, 224, 224]) torch.Size([2]) tensor([0, 1])
```
表示：
```
i = 0
    第 0 个 batch
inputs.shape = [2, 3, 224, 224]
    2：batch_size，一共有 2 张图片
    3：RGB 三通道
    224：图片高度
    224：图片宽度
target.shape = [2]
    2 张图片对应 2 个标签
target = tensor([0, 1])
    第 1 张图片标签为 0（ants）
    第 2 张图片标签为 1（bees）
```
PyTorch 图像 batch 常见格式：
```
[B, C, H, W]
B：Batch Size
C：Channel
H：Height
W：Width
```
