---
title: "PyTorch Module APIs"
createTime: 2026/09/28 00:00:00
permalink: /notes/MachineLearning/pytorch/module-apis.html
---

# PyTorch Module APIs
## 设置模型存放在 cpu/gpu
to() 方法的妙用：根据当前平台自动选择 CUDA、MPS 或 CPU。to(device) 比直接调用 cuda() 更适合跨平台代码。
```python
if torch.cuda.is_available():
    net.to("cuda")  # 原有 Windows / Linux + NVIDIA GPU 的 CUDA 写法
elif torch.backends.mps.is_available():
    net.to("mps")  # macOS Apple Silicon（M 系列芯片）
else:
    net.to("cpu")  # Intel Mac 或没有可用 GPU 时的 fallback
```
这一版也适配了 MacOS。
## 获取模型参数
保存模型参数要用到 state_dict 函数。
```python
class TinnyCNN(nn.Module):
    def __init__(self, cls_num=2):
        super(TinnyCNN, self).__init__()
        # 设计了两层的网络
        self.convolution_layer = nn.Conv2d(1, 1, kernel_size=(3, 3))
        self.fc = nn.Linear(36, cls_num) # 默认 cls_num 为2，这是一个 36 -> 2 的线性层
    def forward(self, x):
        x = self.convolution_layer(x)
        # 把卷积后的特征图展开成一维向量,摊平
        x = x.view(x.size(0), -1) # x.size(0)就是 batch size， -1表示剩下维度自动计算
        out = self.fc(x)
        return out
model = TinnyCNN(2)
state_dict = model.state_dict()
for key, parameter_value in state_dict.items():
    print(key)
    print(parameter_value, end="\n\n")
```
输出就是：
```bash
convolution_layer.weight
tensor([[[[-0.0586,  0.2958,  0.0794],
          [-0.0975, -0.1863, -0.2886],
          [ 0.0428, -0.0775, -0.0212]]]])
convolution_layer.bias
tensor([-0.0820])
fc.weight
tensor([[ 0.1165,  0.0235, -0.0438,  0.1110,  0.1489, -0.0167, -0.0951,  0.0331,
          0.1470, -0.0072,  0.0420, -0.0689,  0.0828,  0.0833,  0.1161,  0.0625,
          0.1383,  0.0415, -0.0853, -0.0155, -0.0974,  0.1354, -0.0970,  0.1201,
          0.0051,  0.1168, -0.0631, -0.0012, -0.1244,  0.0307,  0.1647,  0.1610,
         -0.0011, -0.0883,  0.0176, -0.1347],
        [ 0.1008, -0.0250,  0.0644,  0.0470,  0.1096, -0.0654,  0.0835,  0.1284,
         -0.0624,  0.0111, -0.0449, -0.1071,  0.1286, -0.1086, -0.0395,  0.1416,
          0.1321, -0.1113, -0.0998, -0.0708, -0.0049,  0.0485, -0.1228,  0.0476,
          0.1246,  0.1118, -0.1652, -0.0927, -0.1385,  0.1040, -0.1084, -0.0119,
         -0.0311, -0.0928, -0.0599,  0.1003]])
fc.bias
tensor([0.0320, 0.1332])
```
这些就是两层网络的 weight 和 bias。
state_dict() 就是模型参数的“字典”，key 是参数名，value 是参数具体数值，常用于模型保存和加载。
## 加载模型参数
**load_state_dict**：将参数字典中的参数复制到当前模型中。这里的复制要求key要一一对应，若key对不上，自然模型不知道要把这个参数放到哪里去。绝大多数开发者都会在 load_state_dict 这里遇到过报错。
```python
model = TinnyCNN(2)
state_dict_tinnycnn = model.state_dict()
state_dict_tinnycnn["convolution_layer.weight"][0, 0, 0, 0] = 12345. # 假设经过训练，权重参数发现变化
model.load_state_dict(state_dict_tinnycnn)  # 再次查看
for key, parameter_value in model.state_dict().items():
    print(key)
    print(parameter_value, end="\n\n")
```
## 管理模型的 modules, parameters, sub_module
- parameters：返回一个迭代器，迭代器可抛出 Module 的所有 parameter 对象
- named_parameters：作用同上，不仅可得到 parameter 对象，还会给出它的名称
- modules：返回一个迭代器，迭代器可以抛出 Module 的所有 Module 对象，注意：模型本身也是 module，所以也会获得自己。
- named_modules：作用同上，不仅可得到 Module 对象，还会给出它的名称
- children：作用同modules，但不会返回 Module 自己。
- named_children：作用同named_modules，但不会返回 Module 自己。
## 获取某个参数或 submodule
当想查看某个部分数据时，可以通过 get_xxx 方法获取模型特定位置的数据，可获取 parameter、submodule，使用方法也很简单，只需要传入对应的 name 即可。
