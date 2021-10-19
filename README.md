#SCURM_ROS_CORE

四川大学RoboMaster火锅战队上位机ROS2框架，便于各个模块（自瞄，大符，决策）解耦，加速算法开发流程。

本项目仿照UESTC开源工具栈[
RoboMaster OSS](https://github.com/robomaster-oss)进行开发。

> master_base为整体框架的基础模块提供串口通讯、摄像头驱动和部分常用工具包。

## 模块说明

|模块|说明|
|:-:|:-:|
|`rm_base`|通讯工具包：可用于上下位机串口通讯和网络UDP通讯|
|`rm_cam`|摄像头驱动包：对常用相机SDK提供ROS通讯的封装|
|`rm_interfaces`|接口定义包：对框架的消息接口进行定义|
