# 3DLineDetection

## 原始文档

原始代码链接：https://github.com/xiaohulugo/3DLineDetection

论文：2019 Fast 3D Line Segment Detection From Unorganized Point Cloud. You can also find it [here](https://github.com/blue-stone-j/papers/blob/main/2019%20Fast%203D%20Line%20Segment%20Detection%20From%20Unorganized%20Point%20Cloud.pdf)

These material may help you learn this code. {[csdn](https://blog.csdn.net/m0_37957160/article/details/106823271), [zhihu](https://zhuanlan.zhihu.com/p/353878655)}

## 简介

包含了论文和注释后的代码, 便于理解论文和代码的原理和工作流程。This code can be compiled by command `./cbuild.sh`. But there's not guarantee to run normally. I didn't make any changes to this code.

## 代码特性

1 平面和直线都是结构化的几何形状, 主要适用于人造场景, 例如城市道路, 港口。 露天矿场和野外的主要场景特征为颠簸路面和不规则斜坡, 适用性不强

2 代码给出了整个点云处理流程, 但没有使用 ros autoware 等框架, 应根据需求适配框架

3 kd树部分的代码是一个较为独立的模块, 梳理算法和代码流程时可以略过这一部分, 不会影响对程序的理解
