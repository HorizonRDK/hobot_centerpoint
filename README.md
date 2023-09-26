# 功能介绍

激光雷达目标检测算法是使用地平线[OpenExplorer](https://developer.horizon.ai/api/v1/fileData/horizon_j5_open_explorer_cn_doc/hat/source/examples/centerpoint.html)在[nuscenes](https://www.nuscenes.org/nuscenes)数据集上训练出来的`CenterPoint`算法模型。

算法输入为32线激光雷达点云数据，输出信息包括目标的3D检测框、置信度、类别。支持的目标检测类型包括car、truck、bus、barrier、motorcycle、pedestrian共六大类别。

此示例使用本地激光雷达点云文件作为输入，利用BPU进行算法推理，发布包含点云数据、目标检测框和朝向的渲染图片消息，在PC端浏览器上渲染显示算法结果。

# 物料清单


# 使用方法

## 功能安装

在RDK系统的终端中运行如下指令，即可快速安装：

```bash
sudo apt update
sudo apt install -y tros-hobot-centerpoint
sudo apt install -y tros-websocket
```

## 准备回灌数据集

在RDK系统的终端中运行如下指令，下载并解压数据集：

```shell
# 板端下载回灌的点云文件
wget http://archive.sunrisepi.tech/TogetheROS/data/hobot_centerpoint_data.tar.gz

# 解压缩
mkdir config
tar -zxvf hobot_centerpoint_data.tar.gz -C config
# 解压完成后数据在config/hobot_centerpoint_data路径下
```

## 启动算法和图像可视化

在RDK系统的终端中运行如下指令，启动算法和可视化：

```shell
# 配置tros.b环境
source /opt/tros/setup.bash

# 启动websocket服务
ros2 launch websocket websocket_service.launch.py

# 启动launch文件
ros2 launch hobot_centerpoint hobot_centerpoint_websocket.launch.py lidar_pre_path:=config/hobot_centerpoint_data
```

启动成功后，打开同一网络电脑的浏览器，访问RDK的IP地址http://IP:8000（IP为RDK的IP地址），即可看到算法可视化的实时效果：

![centerpoint](img/centerpoint.gif)


# 接口说明

## 话题

| 名称         | 消息类型                             | 说明                                     |
| ------------ | ------------------------------------ | ---------------------------------------- |
| /hobot_centerpoint  | sensor_msgs/msg/Image                | 周期发布的图像话题，jpeg格式             |

## 参数

| 名称                         | 参数值                                          | 说明                                               |
| ---------------------------- | ----------------------------------------------- | -------------------------------------------------- |
| lidar_pre_path                 | 使用回灌数据集实际所在路径，默认./config/hobot_centerpoint_data | 回灌数据集路径                         |
| lidar_list_file                 | 使用回灌数据集实际文件名，默认./config/nuscenes_lidar_val.lst | 回灌数据列表                         |
| is_loop                 | True（默认）/False | 是否发布渲染后图片                         |

# 常见问题
