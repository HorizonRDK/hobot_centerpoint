Getting Started with Hobot CenterPoint Node
=======


# 功能介绍

激光雷达目标检测算法示例使用激光雷达点云作为输入，利用BPU进行算法推理，并发布包含点云数据，检测框和目标朝向的渲染图片msg。

CenterPoint为地平线开源的雷达检测模型。模型输出信息包括目标的3D检测框、置信度、类别。支持的目标检测类型包括car、truck、bus、barrier、motorcycle、pedestrian等六大类别。

# 开发环境

- 编程语言: C/C++
- 开发平台: J5
- 系统版本：Ubuntu 20.0.4
- 编译工具链:Linux GCC 9.3.0/Linaro GCC 9.3.0

# 编译

- J5版本：支持在J5 Ubuntu系统上编译和在PC上使用docker交叉编译。

## J5 Ubuntu系统上编译 J5版本

1、编译环境确认

- 板端已安装J5 Ubuntu系统。

- 当前编译终端已设置TROS·B环境变量：`source /opt/tros/setup.bash`。

- 已安装ROS2软件包构建系统ament_cmake。安装命令：`apt update; apt-get install python3-catkin-pkg; pip3 install empy`

- 已安装ROS2编译工具colcon。安装命令：`pip3 install -U colcon-common-extensions`

2、编译

- 编译命令：`colcon build --packages-select hobot_centerpoint`

## docker交叉编译 J5版本

1、编译环境确认

- 在docker中编译，并且docker中已经编译好TROS·B。docker安装、交叉编译、TROS·B编译和部署说明详见[TogetheROS.Bot用户手册](https://developer.horizon.ai/api/v1/fileData/documents_tros/quick_start/cross_compile.html#)。

2、编译

- 编译命令：

  ```shell
  bash robot_dev_config/build.sh -p J5 -s hobot_centerpoint
  ```

# 使用介绍

## 参数
|         字段         |      类型    |         描述       | 是否必须 |         默认值              |
| -------------------- | ----------- | ------------------ | ------- | --------------------------- |
| preprocess_config     |   string    |   预处理配置文件   |   否     |config/centerpoint_preprocess_5dim.json|
| model_file            |   string    |centerpoint模型文件路径|   否  |     config/model/model.hbm  |
| lidar_list_file       |   string    |本地回灌激光雷达文件列表|   否   |config/nuscenes_lidar/nuscenes_lidar.lst |
| is_show             |     bool      | 是否进行渲染展示    |     否  |  true               |
| is_loop             |     bool      | 本地回灌是否循环进行  |   否  |   true                |
| pub_topic_name      |     string    | 发布的可视化图片话题名  |   否  | /hobot_centerpoint   |
| lidar_pre_path      |     string    | 回灌的二进制文件所在路径 | 否  | config/hobot_centerpoint_data               |

## J5 Ubuntu系统上运行

**数据集处理**
数据集下载参考[工具链文档](https://developer.horizon.ai/api/v1/fileData/horizon_j5_open_explorer_cn_doc/runtime/source/ai_benchmark/source/ai-benchmark.html#nuscenes), v1.0-mini版本的数据集需要下载的内容大约有10G
meta文件夹构造参考[地平线Centerpoint参考算法](https://developer.horizon.ai/forumDetail/163807121354434630)3.1.4章节 meta文件夹构建
```shell
#下载OE开发包
wget -c ftp://j5ftp@vrftp.horizon.ai/OpenExplorer/v1.1.52a_release/horizon_j5_open_explorer_v1.1.52a-py38_20230605.tar.gz --ftp-password=j5ftp@123$%
#解压OE
tar -zxvf horizon_j5_open_explorer_v1.1.52a-py38_20230605.tar.gz -C v1.1.52a

#下载Ubuntu20.04 GPU Docker镜像并加载
wget -c ftp://j5ftp@vrftp.horizon.ai/OpenExplorer/v1.1.52a_release/docker_openexplorer_ubuntu_20_j5_gpu_v1.1.52a.tar.gz --ftp-password=j5ftp@123$%
sudo docker load --input docker_openexplorer_ubuntu_20_j5_gpu_v1.1.52a.tar.gz

#启动docker挂载目录
sudo nvidia-docker run -it --shm-size="15g" -v v1.1.52a:/WORKSPACE openexplorer/ai_toolchain_ubuntu_20_j5_gpu:v1.1.52a /bin/bash

#处理数据集中的激光雷达点云数据
python3 centerpoint_preprocess.py --data-path=./Nuscenes --save-path=./nuscenes_lidar_val
#处理脚本在OE包中的路径：OE/ddk/samples/ai_benchmark/j5/qat/tools/eval_preprocess/centerpoint_preprocess.py
```

**板端下载回灌文件**
这里提供了处理好的回灌数据的下载方式，数据集版本v1.0-mini
```shell
# 板端下载处理好的二进制雷达文件
wget http://archive.sunrisepi.tech/TogetheROS/data/hobot_centerpoint_data.tar.gz

# 解压缩
mkdir config
tar -zxvf hobot_centerpoint_data.tar.gz -C config
# 解压完成后数据在config/hobot_centerpoint_data路径下
```

**使用本地数据集回灌**

```shell
# 配置TogetheROS·Bot环境
source /opt/tros/setup.bash

# 启动运行脚本，并指定数据集路径
ros2 launch hobot_centerpoint hobot_centerpoint_websocket.launch.py lidar_pre_path:=config/hobot_centerpoint_data
```

PC的WEB端输入板端IP地址`http://IP:8000`，展示回灌结果和实时渲染：
![image](./config/render.jpg)