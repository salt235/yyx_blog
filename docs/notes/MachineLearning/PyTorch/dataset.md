---
title: "PyTorch Dataset"
createTime: 2026/09/26 00:00:00
permalink: /notes/MachineLearning/pytorch/dataset.html
---

# PyTorch Dataset
pytorch 提供的 torch.utils.data.Dataset 类是一个抽象基类，供用户继承，编写自己的 dataset，实现对数据的读取。在 Dataset 类的编写中必须要实现的两个函数是 `__getitem__()` 和 `__len__()` 。
单个样本可在这里完成 [图像变换](/notes/MachineLearning/pytorch/transforms.html)，之后由 [DataLoader](/notes/MachineLearning/pytorch/dataloader.html) 读取并拼成 batch。
- **getitem**：需要实现读取一个样本的功能。通常是传入索引（index，可以是序号或key），然后实现从磁盘中读取数据，并进行预处理（包括online的数据增强），然后返回一个样本的数据。数据可以是包括模型需要的输入、标签，也可以是其他元信息，例如图片的路径。**getitem返回的数据会在dataloader中组装成一个batch**。即，通常情况下是在dataloader中调用Dataset的**getitem**函数获取一个样本。
- **len**：返回数据集的大小，数据集的大小也是个重要的信息，它在dataloader中也会用到。如果这个函数返回的是0，dataloader会报错："ValueError: num_samples should be a positive integer value, but got num_samples=0"
```python
class MyDataset(Dataset):
    def __init__(...):   # 初始化：记录样本索引
    def __getitem__(self, index):  # 给定编号，读取一个样本
    def __len__(self):   # 样本总数
```
编写 Dataset 的流程主要有以下几种：
- 第一个：数据的划分及标签在txt中（略）
- 第二个：数据的划分及标签**在文件夹**中体现
- 第三个：数据的划分及标签**在csv**中
## 1. 标签与划分体现在文件夹中
目录结构示例：
```
dataset/
├── train/
│   ├── covid-19/
│   └── no-finding/
└── valid/
    ├── covid-19/
    └── no-finding/
```
核心思想：
```
文件夹名 → 标签
train / valid 文件夹 → 数据集划分
```
Dataset 核心：
```python
class COVID19Dataset_2(Dataset):
	# init 负责读取数据，生成一个列表
    def __init__(self, root_dir, transform=None):
        self.root_dir = root_dir
        self.transform = transform
        self.img_info = [] # 这个列表，就是用来存放(path_img, lable_int)这些元组
        self.str_2_int = {
            "no-finding": 0,
            "covid-19": 1
        } # 这是一个字典，来把文件夹名称对应为标签
        self._get_img_info()
	# init 的一个方法，来读取数据建表
    def _get_img_info(self):
	    # 遍历文件夹的操作
	    # root  → 当前正在遍历的文件夹路径
		# dirs  → 当前文件夹里的子文件夹列表，这里其实没用到
		# files → 当前文件夹里的文件列表
        for root, dirs, files in os.walk(self.root_dir):
            for file in files:
                if file.endswith("png") or file.endswith("jpeg"):
                    path_img = os.path.join(root, file) # 路径拼接
                    sub_dir = os.path.basename(root) # 获取当前文件夹名
                    label_int = self.str_2_int[sub_dir] # 映射为标签
                    self.img_info.append((path_img, label_int)) # 加入列表中
	# getitem 负责根据索引，拿取数据
    def __getitem__(self, index):
        path_img, label = self.img_info[index]
        img = Image.open(path_img).convert('L') # 转换为灰度图
        if self.transform:
            img = self.transform(img)
        return img, label
    def __len__(self):
        return len(self.img_info)
```
最终：
```
self.img_info = [
    (图片路径, 标签),
    ...
]
```
小记（类中的方法命名原则）：
```
__xxx__  → Python 特殊方法，通常有固定含义，会被自动调用
_xxx     → 普通方法，只是约定“内部使用”
xxx      → 普通公开方法
```
## 2. 标签与划分体现在 CSV 中
CSV 示例：
```
img-name,label,set-type
001.png,0,train
002.png,1,train
003.png,0,valid
```
核心思想：
```
img-name  → 图片名
label     → 标签
set-type  → train / valid
```
Dataset 核心：
```python
class COVID19Dataset_3(Dataset):
	# mode 这里分为 train 和 valid，具体要看 CSV 里面怎么写
    def __init__(self, root_dir, path_csv, mode, transform=None):
        self.root_dir = root_dir
        self.path_csv = path_csv
        self.mode = mode
        self.transform = transform
        self.img_info = []
        self._get_img_info()
    def _get_img_info(self):
	    # df 就是 data frame，pd 是 pandas
        df = pd.read_csv(self.path_csv)
        # 只保留当前 mode 的数据，例如只要 train
        df.drop(
            df[df["set-type"] != self.mode].index,
            inplace=True # 直接在原来的 df 上修改，不再生成新的
        )
        df.reset_index(inplace=True) # 重置编号
        for idx in range(len(df)):
            path_img = os.path.join(
                self.root_dir,
                df.loc[idx, "img-name"]
            ) # 拼接路径，loc 是 df 里面的单元格定位操作
            label = int(df.loc[idx, "label"])
            self.img_info.append((path_img, label))
    def __getitem__(self, index):
        path_img, label = self.img_info[index]
        img = Image.open(path_img).convert('L')
        if self.transform:
            img = self.transform(img)
        return img, label
    def __len__(self):
        return len(self.img_info)
```
Pandas 需要认识的语法：
```
pd.read_csv(path)          # 读取 CSV
df["列名"]                 # 获取一列
df[条件]                   # 条件筛选
df.drop(...)               # 删除行
df.reset_index(...)        # 重置索引
df.loc[行, "列名"]         # 获取某个单元格
```
## 3. 两种方式的本质
```
文件夹形式：
文件夹结构
→ 获取图片路径和标签
→ self.img_info
CSV形式：
CSV表格
→ 获取图片路径和标签
→ self.img_info
```
最终目的都是构造：
```
self.img_info = [
    (path, label),
    (path, label),
    ...
]
```
之后由：
```
__getitem__()   # 根据 index 读取一个样本
__len__()       # 返回样本数量
```
完成 Dataset 的基本功能。
## 4. 一些常用 APIs
### concat
在实际项目中，数据的来源往往是多源的，可能是多个中心收集的，也可能来自多个时间段的收集，很难将可用数据统一到一个数据形式。通常有两种做法，一种是固定一个数据形式，所有获取到的数据经过整理，变为统一格式，然后用一个Dataset即可读取。还有一种更为灵活的方式是为每批数据编写一个Dataset，然后使用torch.utils.data.ConcatDataset类将他们拼接起来，这种方法可以灵活的处理多源数据，也可以很好的使用别人的数据及Dataset。
![image.png](https://cdn.jsdelivr.net/gh/salt235/tuchuang/macImg/20260926212839875.png)
### subset
subset可根据**指定的索引**获取子数据集，Subset也是Dataset类，同样包含 `__len__` 和 `__getitem__`。
```python
    def __init__(self, dataset: Dataset[T_co], indices: Sequence[int]) -> None:
        self.dataset = dataset
        self.indices = indices
    def __getitem__(self, idx):
        if isinstance(idx, list): # 判断 idx 是不是 list 类型
            return self.dataset[[self.indices[i] for i in idx]]
        return self.dataset[self.indices[idx]]
    def __len__(self):
        return len(self.indices)
```
#### 参数类型补充
```python
dataset: Dataset[T_co]
```
表示 `dataset` 参数必须是一个 `Dataset` 或其子类对象。
```python
indices: Sequence[int]
```
表示 `indices` 是一个“整数序列”，用于指定要选取哪些样本。
例如：
```python
indices = [0, 2, 4]
```
表示从原 Dataset 中取：
```
第 0、2、4 个样本
```
### random_split
该函数的功能是随机的将dataset划分为多个不重叠的子集，适合用来划分训练、验证集（不过不建议通过它进行，因为对用户而言，其划分不可见，不利于分析）。
