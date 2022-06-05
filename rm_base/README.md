# RM_BASE

`rm_base`串口通信模块。

定义通信协议，为上位机（nx/nuc）与下位机（stm32）的串口通信提供ROS节点。
## 环境 
- ROS2-galactic, CMake 3.8, gcc 9.4.0, Ubuntu 20.04

## 包格式:
- 32位包格式:

|HEAD|DATA|BCC|TAIL|
|-|-|-|-|
|[0]|[1-29]|[30]|[31]|
|包头|数据位|校验位|包尾|
|0xff|见下文详细说明|暂为bcc异或校验|0x0d|

## 通信协议
### **上位机->下位机** 

-  1.步兵、英雄云台控制帧

|数据|说明|type(数据位)|
|-|-|-|
|tid|上位机帧编号|int32【1-4】|
|yaw|目标yaw|float32【5-8】|
|pitch|目标pitch|float32【9-12】|
|gyro_yaw|陀螺仪读出的当前yaw|float32【13-16】|
|angular_velocity_yaw|yaw角速度|float32【17-20】|
|gyro_pitch|陀螺仪读出的当前pitch|float32【21-24】|
|angular_velocity_pitch|pitch角速度|float32【25-28】|


- 2.哨兵云台控制帧

|数据|说明|type(数据位)|
|-|-|-|
|tid|上位机帧编号|int32【1-4】|
|type|哨兵状态（0x2a自瞄,0x3a巡逻,0x4a遥控器）|unsigned char【5】|
|shoot|发弹 0x4b发射|unsigned char【6】|
|yaw|偏移yaw|float32【7-10】|
|pitch|偏移pitch|float32【11-14】|

- 2.哨兵上下云台通信帧
|数据|说明|type(数据位)|
|-|-|-|
|tid|上位机帧编号|int32【1-4】|
|type|哨兵（0x5a 转发给另一个云台）|unsigned char【5】|
|num|当前识别到车的编号label|int32【6-9】|

<!-- - 3.(不需要使用)时间戳同步帧(下位机接收到同步帧后原封不动返回)

|数据|说明|type(数据位)|
|-|-|-|
|tid|下位机帧编号|int32【1-4】1-10，同步10次取平均|
|cmd|0xe1|unsigned char【5】|
|time_stamp|时间戳|float64/double【6-13】| -->

### **下位机->上位机** 

#### 包种类cmd位（位于帧的[5]位）
设置判断位，判断数据包的作用
|变量名|帧种类|数据|
|-|-|-|
|ChangeMode|模式帧|0xa1|
|GetShootSpeed|射速帧|0xb1|
|ChangeColor|颜色帧|0xc1|
<!-- |GimbalAngleControl|姿态帧|0xd1| -->
- 1.模式帧

|数据|说明|type(数据位)|
|-|-|-|
|tid|下位机帧编号|int32【1-4】4294967295-10|
|cmd|0xa1|unsigned char【5】|
|mode|战斗模式|unsigned char【6】:（自瞄 : 0x01、 小符: 0xbb、 大符： 0xcc、手动: 0x00）|
<!-- |GyroQuaternions|当前姿态四元数 |float32 ：Q1【14-17】Q2【18-21】Q3【22-25】Q4【26-29】| -->

- 2.射速帧

|数据|说明|type(数据位)|
|-|-|-|
|tid|下位机帧编号|int32【1-4】4294967295-9|
|cmd|0xb1|unsigned char【5】|
|velocity|子弹发射速度|int32【6-9】|
<!-- |GyroQuaternions|当前姿态四元数 |float32 ：Q1【14-17】Q2【18-21】Q3【22-25】Q4【26-29】| -->

- 3.（我方）颜色帧

|数据|说明|type(数据位)|
|-|-|-|
|tid|下位机帧编号|int32【1-4】4294967295-8|
|cmd|0xc1|unsigned char【5】|
|color|我方颜色|unsigned char【6】:（red : 0x1c/ blue : 0xbc）|
<!-- |GyroQuaternions|当前姿态四元数 |float32 ：Q1【14-17】Q2【18-21】Q3【22-25】Q4【26-29】| -->

<!-- 已替换为独立陀螺仪IMU
 - 4.纯姿态帧

|数据|说明|type(数据位)|
|-|-|-|
|tid|下位机帧编号|int32【1-4】0~(4294967295-10)|
|cmd|0xd1|unsigned char【5】|
|GyroQuaternions|当前姿态四元数 |float32 ：Q1【14-17】Q2【18-21】Q3【22-25】Q4【26-29】| -->

<!-- - 5.时间戳同步帧(下位机接收到同步帧后原封不动返回)

|数据|说明|type(数据位)|
|-|-|-| 
|tid|下位机帧编号|int32【1-4】1-10，同步10次取平均|
|cmd|0xe1|unsigned char【5】|
|time_stamp|时间戳|float64/double【6-13】| -->

## 环境搭建
### 1.安装ROS2（https://docs.ros.org/en/galactic/Installation/Ubuntu-Install-Debians.html） ，安装desktop版本
### 2. 初始化：workspace and packages init
``` shell
echo 'source /opt/ros/galactic/setup.bash' >> ~/.bashrc

mkdir -p ~/XXX/scu_rm_ros
cd ~/XXX/scu_rm_ros

git clone git@e.coding.net:scurm/2022-vision/master-base.git 
(或者直接解压master-base.zip包)

colcon build --symlink-install
```

## 在launch文件中启动node，在yaml文件中修改参数param
serial_name：使用的串口名，serial_send：串口发送，serial_recv：串口接收

·默认           
``` python
                {"serial_name": "/dev/USBtty0"},
                {"serial_send": False},
                {"serial_recv": True}
```
  
表示上位机仅接收下位机数据，此时mode=0x00；当mode=0x01表示开启自瞄时，程序开启发送模块，发送自瞄需要的数据

·Debug测试     
 ``` python
                {"serial_name": "/dev/USBtty0"},
                {"serial_send": True},
                {"serial_recv": True}
```
  开启DEBUG模式，接收发送
```
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Debug
```

# 测试
结点启动终端：
```
ros2 launch rm_base test_serial_recv.py
ros2 launch rm_base test_serial_send.py
```
（另起一个终端）测试终端，模拟自瞄结点广播topic：
```
ros2 topic pub /cmd_gimbal rm_interfaces/msg/GimbalCmd "{position: {pitch: 3.0, yaw: 2.0}}"
```


## run指令建立结点
```
ros2 run rm_base simple_robot_base --ros-args --remap __node:=【结点名】   (ps：等号与节点名中间不能有空格)
ros2 run rm_base simple_robot_base --ros-args --remap __node:=send
ros2 run rm_base simple_robot_base --ros-args --remap __node:=recv

```

## 使用udev给与USB设备永久权限
参考链接 https://blog.csdn.net/m0_38144614/article/details/121297159
（否则每次串口硬件插拔或者是ubuntu系统休眠都需要重新给权限
如果遇到找不到串口的错误，可以先ls /dev找到对应串口名（一般为/dev/ttyUSB或者ttyACM）
然后给权限 ```sudo chmod +777 /dev/ttyxxx```）



# 开发日志

## （已解决）11/2 调试32位packet发送接收时产生bug， 发送部分将16次的packet一次发送，导致接收只能接收到16个包中的一个

```
vofa+ 收到的包 每隔16s才接收一次
[22:38:03.409] FF 95 00 00 00 C1 BB 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 EF 0D FF 96 00 00 00 C1 BB 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 EC 0D FF 97 00 00 00 C1 BB 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 ED 0D FF 98 00 00 00 C1 BB 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 E2 0D FF 99 00 00 00 C1 BB 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 E3 0D FF 9A 00 00 00 C1 BB 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 E0 0D FF 9B 00 00 00 C1 BB 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 E1 0D FF 9C 00 00 00 C1 BB 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 E6 0D FF 9D 00 00 00 C1 BB 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 E7 0D FF 9E 00 00 00 C1 BB 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 E4 0D FF 9F 00 00 00 C1 BB 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 E5 0D FF A0 00 00 00 C1 BB 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 DA 0D FF A1 00 00 00 C1 BB 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 DB 0D FF A2 00 00 00 C1 BB 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 D8 0D FF A3 00 00 00 C1 BB 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 D9 0D FF A4 00 00 00 C1 BB 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 DE 0D 
[22:38:19.409] FF A5 00 00 00 C1 BB 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 DF 0D FF A6 00 00 00 C1 BB 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 DC 0D FF A7 00 00 00 C1 BB 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 DD 0D FF A8 00 00 00 C1 BB 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 D2 0D FF A9 00 00 00 C1 BB 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 D3 0D FF AA 00 00 00 C1 BB 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 D0 0D FF AB 00 00 00 C1 BB 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 D1 0D FF AC 00 00 00 C1 BB 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 D6 0D FF AD 00 00 00 C1 BB 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 D7 0D FF AE 00 00 00 C1 BB 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 D4 0D FF AF 00 00 00 C1 BB 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 D5 0D FF B0 00 00 00 C1 BB 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 CA 0D FF B1 00 00 00 C1 BB 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 CB 0D FF B2 00 00 00 C1 BB 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 C8 0D FF B3 00 00 00 C1 BB 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 C9 0D FF B4 00 00 00 C1 BB 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 CE 0D
```

## 11/7 串口发送频率极限测试 
bps = 115200
100000hz 拉满， 但实际上只能达到**2ms~4ms**左右的延迟， 也就是333hz

### vofa+ 接收端
``` 
[19:04:17.860] FF 7A 0A 00 00 C1 BB 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 0A 0D 
[19:04:17.864] FF 7B 0A 00 00 C1 BB 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 0B 0D 
[19:04:17.867] FF 7C 0A 00 00 C1 BB 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 0C 0D 
[19:04:17.870] FF 7D 0A 00 00 C1 BB 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 0D 0D 
[19:04:17.873] FF 7E 0A 00 00 C1 BB 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 0E 0D 
[19:04:17.876] FF 7F 0A 00 00 C1 BB 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 0F 0D 
[19:04:17.878] FF 80 0A 00 00 C1 BB 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 F0 0D 
[19:04:17.881] FF 81 0A 00 00 C1 BB 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 F1 0D 
[19:04:17.885] FF 82 0A 00 00 C1 BB 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 F2 0D 
[19:04:17.888] FF 83 0A 00 00 C1 BB 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 F3 0D 
[19:04:17.890] FF 84 0A 00 00 C1 BB 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 F4 0D 
```
### send node 发送端
```
[simple_robot_base-1] [INFO] [1636283057.882132436] [send]: packet[2690] Send 
[simple_robot_base-1] [INFO] [1636283057.882186091] [send]: SEND-ID: '2690'
[simple_robot_base-1] [INFO] [1636283057.884974350] [send]: packet[2691] Send 
[simple_robot_base-1] [INFO] [1636283057.885084151] [send]: SEND-ID: '2691'
[simple_robot_base-1] [INFO] [1636283057.887884238] [send]: packet[2692] Send 
[simple_robot_base-1] [INFO] [1636283057.887949062] [send]: SEND-ID: '2692'
```


## 11/14 与电控联调
上位机发-->下位机解、发-->上位机解，
**115200** 波特率延迟只能到达 **6 ms** 左右， **1152000** 波特率延迟只能到达 **4 ms** 左右，
上位机发-->串口杜邦线直连-->上位机解，
**1152000** 波特率延迟能达到 **600-700 us** 左右

## 11/18 联调
上位机发-->下位机解、发-->上位机解，
 **1152000** 波特率延迟能到达 **1 ms** 左右
 不稳定，在800us-2000us波动（一次发收400-1000us）
 较为合理，但表明有丢包现象



        // x.push_back(position3d(0,0));
        // y.push_back(position3d(1,0));
        // z.push_back(position3d(2,0));
        
        // if(x.size() >= 20)
        // {
        //     double x_all = 0;
        //     double y_all = 0;
        //     double z_all = 0;
        //     for(int i=0;i<x.size();i++)
        //     {
        //         x_all += x[i];
        //         y_all += y[i];
        //         z_all += z[i];
        //     }
        //     double x_all_t = 0;
        //     double y_all_t = 0;
        //     double z_all_t = 0;
        //     for(int i=0;i<x.size();i++)
        //     {
        //         x_all_t += (x[i] - x_all/x.size())*(x[i] - x_all/x.size());
        //         y_all_t += (y[i] - y_all/x.size())*(y[i] - y_all/x.size());
        //         z_all_t += (z[i] - z_all/x.size())*(z[i] - z_all/x.size());
        //     }
        //     double x_off = x_all_t/x.size();
        //     double y_off = y_all_t/x.size();
        //     double z_off = z_all_t/x.size();
        //     RCLCPP_INFO(node_->get_logger(),"\n\n\n\n\n x_off: %f, y_off: %f, z_off: %f \n\n\n\n\n\n", x_off ,y_off ,z_off);
        // }