# transformer

>  最早的transformer = encoder + decoder

## Layer Normalization

Batch Normalization (BN) 虽然在 CNN 上效果很好，但在以下场景存在致命缺陷，促使了 LN 的诞生：

1. **RNN/LSTM 等序列模型**：序列长度不固定。如果在不同时间步做 BN，统计量会波动极大且难以维护。
2. **小 Batch Size (甚至 Batch=1)**：大模型（如 LLM）训练时显存受限，Batch Size 往往很小，此时 BN 的统计量估算失效。

LN 通过**抛弃对 Batch 维度的依赖**，完美解决了上述问题。

### 1. LN 和 BN 的区别

很像，但是：

假设输入张量形状为 $(N, C, H, W)$ 

- **BN (纵向切)**：固定 Channel，跨越 Batch ($N$) 统计。

  在cnn中，可以想象成把一个通道的那张图norm。BN 非常适合 CNN。

- **LN (横向切)**：固定 Sample ($N$)，跨越 Feature ($C, H, W$) 统计。

  在cnn中，可以想象成定一个点，然后竖着和每一个通道的该位置点，合起来一起norm。但是 LN **在 CNN 上表现一般**：在图像分类等任务中，LN 往往打不过 BN。

  - *原因*：图像的特征（Channel）之间差异很大（比如边缘检测 vs 颜色检测），强制把它们拉到一个分布可能会破坏图像的空间/语义信息。

### 2. 对于Transformer，用LN

原因：

1. **序列长度不定**：NLP 句子有长有短，BN 需要对齐长度（Padding），Padding 的 0 值会严重干扰 BN 的均值方差计算。
2. **样本统计不稳**：NLP 数据的 Batch 统计量往往不如图像稳定，且 Batch Size 受显存限制通常较小，BN 效果差。

## encoder

### Bert结构

![早期的一个encoder结构](https://cdn.jsdelivr.net/gh/salt235/tuchuang@main/img/QQ_1767016854328.png)

上图其实就是Bert的结构，Bert本质上就是一个编码器。

输入是四个向量，输出是四个处理后的特征向量（包含了上下文信息，且与x一一对应）。

- 首先对输入的向量，进行**位置编码**，然后送入一个多头的Attention，让模型在处理当前单词时，能够“关注”到句子里的其他单词。

- 然后**Add**: 指的是 Residual Connection (残差连接)。看旁边的箭头，输入直接绕过了 Attention 层加到了输出上。这能防止网络过深导致的退化。同时**Norm**: 指的就是 **Layer Normalization (LN)**。

- 接着进入一个就是一个简单的**全连接网络（MLP）**，对特征进行进一步的非线性变换。

- 最后再次Add和Norm，输出。

BERT 就是一堆 **Transformer Encoder** 叠起来。每个 Encoder 层里有两步核心操作：先做**注意力 (Attention)**，再做**前馈 (Feed Forward)**，每一步做完都要记得 **Add & Norm (残差+层归一化)**。

> 在 BERT/Transformer 的图里，“前馈”就是一个**负责特征提取和非线性变换的“加工厂”**。它没有记忆功能（不存状态），只是单纯地把上一层给它的数据处理好，然后扔给下一层。

## decoder

