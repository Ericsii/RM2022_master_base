# RM_CAM

`rm_cam`相机工具包模块，为虚拟摄像头、mindvision工业摄像头提供ROS节点和topic。


## 功能

- MindVision工业相机驱动：触发并获取工业相机图像，发布成ROS topic(`senser_msgs/msg/Image`)
- 基于图像或视频文件的虚拟摄像头：读取本地文件，并发布成ROS topic
- 自定义ROS的QoS(Quality of Service)：参考rm_interfaces


## 文件说明

- `cam_interfaces.hpp`: 通用相机接口定义
- `cam_server.hpp/cpp`: server模块将图像输出成ROS topic
- `***_cam.hpp/cpp`: 相关相机驱动
- `***_cam_node.hpp/cpp`: ROS顶层模块，整合server和相机驱动实现cam节点