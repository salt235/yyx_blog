---
title: "PyTorch 模型评价指标"
createTime: 2026/09/15 00:00:00
permalink: /notes/MachineLearning/pytorch/evaluation-metrics.html
---

# PyTorch 模型评价指标
[参考文档](https://datawhalechina.github.io/thorough-pytorch/%E7%AC%AC%E9%9B%B6%E7%AB%A0/0.2%20%E8%AF%84%E4%BB%B7%E6%8C%87%E6%A0%87.html#)
## 模型评价指标
### 1. 混淆矩阵
混淆矩阵（也称误差矩阵）是机器学习和深度学习中表示精度评价的一种标准格式，常用n行n列的矩阵形式来表示。其列代表的是预测的类别，行代表的是实际的类标，以一个常见的二分类的混淆矩阵为例。我们会发现二分类的混淆矩阵包括**TP, FP, FN, TN**，其中TP为True Positive，True代表实际和预测相同，Positive代表预测为正样本。同理可得，False Positive (FP)代表的是实际类别和预测类标不同，并且预测类别为正样本，实际类别为负样本；False Negative (FN)代表的是实际类别和预测类标不同，并且预测类别为负样本，实际类别为正样本；True Negative (TP)代表的是实际类别和预测类标相同，预测类别和实际类别均为负样本。
![image.png](https://cdn.jsdelivr.net/gh/salt235/tuchuang/macImg/20260915083848068.png)
举例：假设对100个人进行核酸检测，实际结果为98个阴性，2个阳性，但是我们的模型对核酸检测结果进行预测，预测结果为94个阴性，6个阳性结果，在这里我们定义核酸结果阴性为正样本，核酸结果阳性为负样本。在这个例子中，TP代表实际为阴性且被预测为阴性的数量，共有94人；FP代表实际为阳性，模型预测为阴性的数量，共有0人；FN代表实际为阴性被模型判断为阳性的数量，共有4人；TN代表实际为阳性，被模型识别为阳性的数量，共有2人。
![image.png](https://cdn.jsdelivr.net/gh/salt235/tuchuang/macImg/20260915083957305.png)
这是一个10类标的混淆矩阵。们在拥有混淆矩阵后，可以计算Accuracy，Precision，Recall，F1 Score等衡量模型的评价指标。
### 2. Overall Accuracy
代表了所有预测正确的样本占所有预测样本总数的比例：
$$
\rm{OA} =\frac{\rm{TP+TN}}{\rm{TP+TN+FP+FN}} = \frac{N_{correct}}{N_{total}}
$$
```python
def compute_oa(matrix):
    """
    计算总体准确率,OA=(TP+TN)/(TP+TN+FP+FN)
    :param matrix:
    :return:
    """
    return np.trace(matrix) / np.sum(matrix)
```
### 3. Average accuracy
Average accuracy( AA) 代表的是平均精度的计算，平均精度计算的是每一类预测正确的样本与该类总体数量之间的比值，最终再取每一类的精度的平均值。代码实现中，我们使用numpy的diag将混淆矩阵的对角线元素取出，并且对于混淆矩阵进行列求和，用对角线元素除以求和后的结果，最后对结果计算求出平均值。
```python
def compute_aa(matrix):
    """
    计算每一类的准确率,AA=(TP/(TP+FN)+TN/(FP+TN))/2
    :param matrix:
    :return:
    """
    return np.mean(np.diag(matrix) / np.sum(matrix, axis=1))
```
### 4. Kappa 系数
Kappa系数是一个用于一致性检验的指标，也可以用于衡量分类的效果。它和普通准确率 OA（Overall Accuracy）很像，但比准确率多考虑了一件事：有些预测正确可能只是“碰巧猜对的”。
$$
kappa = \frac{p_o-p_e}{1-p_e}\\
p_o = OA\\
p_e = \frac{\sum_{i} (x_i \cdot x_j)}{(\sum_{j=0}^n\sum_{i=0}^{n} x_{ij})^2}
$$
- $P_o$：实际观察到的一致率，也就是 OA / Accuracy
- $P_e$：随机情况下“碰巧预测正确”的概率
- $\kappa$：排除随机因素后，真正的一致程度
$P_e$ 可能需要理解一下，它是**真实标签和预测结果彼此独立时，仅因为类别比例而碰巧一致的概率。**
```python
def compute_kappa(matrix):
    """
    计算kappa系数
    :param matrix:
    :return:
    """
    oa = self.compute_oa(matrix)
    pe = 0
    for i in range(len(matrix)):
        pe += np.sum(matrix[i]) * np.sum(matrix[:, i])
    pe = pe / np.sum(matrix) ** 2
    return (oa - pe) / (1 - pe)
```
### 5. Recall
Recall也称召回率，代表了实际为正样本并且也被正确识别为正样本的数量占样本中所有为正样本的比例，可以用下述公式进行表示：
$$
\rm{Recall} = \frac{\rm{TP}}{\rm{TP + FN}}
$$
### 6. Precision
Precision也称精准率，代表的是在全部预测为正的结果中，被预测正确的正样本所占的比例，可以用下述公式进行表示
$$
\rm{Precision} = \frac{\rm{TP}}{\rm{TP + FP}}
$$
和Recall不同的是，Precision代表了预测结果中有多少样本是分类正确的。
### 7. F1
$$
\rm{F_1} = 2\cdot\frac{P\times R}{P+R} = \frac{2TP}{FP+FN+2TP}
$$
F1在模型评估中也是一种重要的评价指标，F1可以解释为召回率（Recall）和P（精确率）的加权平均，F1越高，说明模型鲁棒性越好。人们希望有一种更加广义的方法定义F-score，希望可以改变P和R的权重，于是人们定义了$F_{\beta}$，其定义式如下：
$$
\rm{F_{\beta}}=\frac{\left(1+\beta^{2}\right) \times P \times R}{\left(\beta^{2} \times P\right)+R}
$$
- 当 $\beta$ > 1 时，更偏好召回(Recall)
- 当 $\beta$ < 1 时，更偏好精准(Precision)
- 当 $\beta$ = 1 时，平衡精准和召回，即为 F1
当有多个混淆矩阵（多次训练、多个数据集、多分类任务）时，有两种方式估算 “全局” 性能：
- macro 方法：先计算每个 PR，取平均后，再计算 F1
- micro 方法：先计算混淆矩阵元素的平均，再计算 PR 和 F1
### 8. Recall Precision 和 F1
| 指标        | 核心问题                     |
| --------- | ------------------------ |
| Precision | 预测为某动作时，有多少是真的           |
| Recall    | 某动作实际出现时，有多少被识别出来        |
| F1        | Precision 和 Recall 的综合指标 |
### 9. 置信度
在目标检测中，我们通常需要将边界框内物体划分为正样本和负样本。我们使用置信度这个指标来进行划分，当小于置信度设置的阈值判定为负样本（背景），大于置信度设置的阈值判定为正样本.
