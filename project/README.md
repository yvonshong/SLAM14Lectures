# direct_deep_slam
SLAM system using direct method to solve motion estimation and use deep learning (VGG net) to accelerate loop closure.

# framework
1. 帧 Frame
1. 关键帧 KeyFrame
1. 关键帧管理 KeyFrameManagement
1. 位姿估计 DirectSolver
1. 全局BA DirectBA
1. 回环检测 LoopClosing
    1. server 收到图片返回VGG输出结果
    1. client 发送图片给server，并将结果构造KNN，搜索临近关键帧检测回环。

完成部分：
- tensorflow部分
- 直接法BA和直接法求解位姿
未完成：
- 帧管理
- 全局BA