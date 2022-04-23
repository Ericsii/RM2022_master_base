# rm_entity_cam

`rm_entity_cam`提供了适配了`rm_cam`的工业相机驱动，并基于`rm_cam`实现相机ROS节点。

## 依赖

* `rm_interfaces`
* `rm_cam`
* `OpenCV 4.x`

## 使用方式

> 参照`rm_cam`工具包中定义

launch方式运行：

```bash
ros2 launch rm_entity_cam mindvision_cam.launch.py
ros2 launch rm_entity_cam daheng_cam.launch.py
```