---
title: "PyTorch 模型保存、断点续训与微调"
createTime: 2026/09/30 00:00:00
permalink: /notes/MachineLearning/pytorch/training-tips.html
---

# PyTorch 模型保存、断点续训与微调
## 1. 模型的保存和加载
在 pytorch 中，对象就是模型，所以我们常常听到序列化和反序列化，就是将训练好的模型从内存中保存到硬盘里，当要使用的时候，再从硬盘中加载。
### torch.save / torch.load
可以保存整个模型 `torch.save(model, "model.pth")` ，或者模型的参数 `torch.save(model.state_dict(), "model.pth")`，一般保存参数即可。
```python
# 保存的路径
path_state_dict = "resnet50_state_dict_2022.pth"
resnet_50 = models.resnet50(pretrained=False)
# 保存 state_dict，即参数
net_state_dict = resnet_50.state_dict()
torch.save(net_state_dict, path_state_dict)
# 加载参数
resnet_50_new = models.resnet50(pretrained=False)
state_dict = torch.load(path_state_dict)
resnet_50_new.load_state_dict(state_dict)
```
### checkpoint
checkpoint 本质是一个字典，要保存什么自己写。
resume 可以理解为断点续训练。
```python
# checkpoint
checkpoint = {
"model": model_without_ddp.state_dict(),
"optimizer": optimizer.state_dict(),
"lr_scheduler": lr_scheduler.state_dict(),
"epoch": epoch,
}
path_save = "model_{}.pth".format(epoch)
torch.save(checkpoint, path_save)
# resume
checkpoint = torch.load(path_save, map_location="cpu")
model.load_state_dict(checkpoint["model"])
optimizer.load_state_dict(checkpoint["optimizer"])
lr_scheduler.load_state_dict(checkpoint["lr_scheduler"])
start_epoch = checkpoint["epoch"] + 1
```
## 2. Finetune 模型微调
通常，会将模型划分为两个部分
1. feature extractor: 将 fc 层之前的部分认为是一个 feature extractor (特征提取器)
2. classifier: fc 层认为是 classifier
|方法|哪些参数训练|训练速度|数据需求|过拟合风险|
|---|---|--:|--:|--:|
|冻结骨干网络|只训练最后分类层|快|较少|较低|
|全量微调|整个网络都训练|慢|较多|较高|
### 冻结 feature extractor，只训练最后的 classifier
前面的 CNN 已经通过 ImageNet 学到了比较通用的图像特征，比如边缘、纹理、形状，我们直接拿这些特征来用，只重新训练最后的分类器。
冻结所有参数，不再计算梯度：
```python
for param in model.parameters():
    param.requires_grad = False
```
修改最后一层，比如把原来的 1000 分类换成 2 分类：
```python
model.fc = nn.Linear(model.fc.in_features, 2)
```
### 整个网络一起微调，不同层不同学习率
预训练好的 backbone：小一点的学习率，慢慢调整；新换的 fc 层：大一点的学习率，更快学习新任务。
```python
 # 返回的是该层所有参数的内存地址
fc_params_id = list(map(id, resnet18_ft.fc.parameters()))
#遍历model的参数，只要不是需要ignore的，就保留，返回filter对象，在optimizer.py中的add_param_group中有
base_params = filter(lambda p: id(p) not in fc_params_id, resnet18_ft.parameters())
optimizer = optim.SGD([
    {'params': base_params, 'lr': LR},  # 0
    {'params': resnet18_ft.fc.parameters(), 'lr': LR*2}], momentum=0.9)
```
