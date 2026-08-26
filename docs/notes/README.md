---
pageLayout: home
pageClass: notes-home
title: 笔记
permalink: /notes/
config:
  - type: doc-hero
    hero:
      name: 学习笔记
      tagline: 记录学习、科研与工程实践中的知识和经验。
      image: /jufufu.png
  - type: features
    title: 学习/研究
    features:
      - title: 语言学习
        icon: 💻
        details: Java、C++、JavaScript 等编程语言的学习笔记。
        link: /notes/LanguageLearning/
      - title: 机器学习与深度学习
        icon: 🧠
        details: 机器学习、深度学习、CNN 与 Attention 模型。
        link: /notes/MachineLearning/
      - title: 无人机入门
        icon: ✈️
        details: 无人机基础、PX4 飞控以及 ROS1 联合仿真。
        link: /notes/UAV/
      - title: ROS 学习
        icon: 🤖
        details: ROS1 基础、通信机制、运行管理与机器人仿真。
        link: /notes/ROS/
  - type: features
    title: 工程/积累
    features:
      - title: 开发工具
        icon: 🛠️
        details: Git、SSH 等常用开发工具的使用与实践。
        link: /notes/DevTools/
      - title: 科研与项目
        icon: 🧪
        details: 数据集、实验方法、工程实现和项目分析。
        link: /notes/Projects/
      - title: 文献阅读笔记
        icon: 📚
        details: 论文阅读过程中整理的方法、实验与结论。
        link: /notes/Papers/
      - title: 杂项笔记与记录
        icon: 🗂️
        details: 暂未归入专题的调研、汇报和知识记录。
        link: /notes/Misc/
---

<style>
.notes-home {
  --vp-home-hero-image-background-image: radial-gradient(
    circle,
    rgba(231, 183, 79, 0.5) 0%,
    rgba(217, 157, 61, 0.2) 46%,
    transparent 72%
  );
  --vp-home-hero-image-filter: blur(38px);
}

.notes-home .vp-home-doc-hero .image-src {
  filter: drop-shadow(0 14px 24px rgba(196, 142, 48, 0.18));
}

[data-theme="dark"] .notes-home {
  --vp-home-hero-image-background-image: radial-gradient(
    circle,
    rgba(232, 183, 77, 0.4) 0%,
    rgba(204, 145, 49, 0.16) 48%,
    transparent 72%
  );
}

@media (min-width: 960px) {
  .notes-home .vp-home-doc-hero .image-container {
    transform: translate(16px, 16px);
  }
}
</style>
