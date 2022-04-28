# init_tools

自启动脚本

## 使用方法

<!-- 将 `scurm.service` 文件复制到 `/usr/lib/systemd/system/`

```bash
sudo cp scurm.service /usr/lib/systemd/system
``` -->

### 配置启动环境

例如：

将所有 ROS package 放置 `/home/scurm/rm_ws/`

将本目录所有文件放置 `/home/scurm/rm_ws/launch/`

```bash
cp * /home/scurm/rm_ws/launch
```

在该目录下新建文件夹`logs`

```bash
cd /home/scurm/rm_ws/launch
mkdir logs
```

### 设置启动节点

在 `/home/scurm/rm_ws/launch/launch_daemon.sh` 中设置工作目录和ros启动节点
在 `/home/scurm/rm_ws/launch/launch_each.sh` 中设置工作目录和ros启动环境

### 设置自动启动

在 ubuntu startup application 中添加 `launch_daemon.sh`

### 停止自启动进程

```bash
./launch_ext.sh
```