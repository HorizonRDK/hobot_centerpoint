// Copyright (c) 2022，Horizon Robotics.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef INCLUDE_CENTERPOINT_NODE_H_
#define INCLUDE_CENTERPOINT_NODE_H_

#include <memory>
#include <vector>
#include <string>
#include <thread>

#include "jsonutil.h"
#include "dnn_node/dnn_node.h"
#include "dnn_node/util/image_proc.h"
#include "preprocess.h"
#include "postprocess.h"
#include "centerpoint_publisher.h"
#include "utils/pc_queue.h"


namespace hobot {
namespace centerpoint {

struct CenterPointNodeOutput : public hobot::dnn_node::DnnNodeOutput {
  // std::vector<uint8_t> lidar_data;
  std::string lidar_files; // 二进制文件
};

// 输入队列数据
struct InputData {
  std::vector<std::shared_ptr<DNNTensor>> input_tensors;
  // std::vector<uint8_t> lidar_data;
  std::string lidar_files;
};

class CenterPoint_Node : public hobot::dnn_node::DnnNode {
public:
  CenterPoint_Node(const std::string& node_name = "centerpoint_node",
                const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

protected:
  // 实现基类的纯虚接口，用于配置Node参数
  int SetNodePara() override;
  // 实现基类的虚接口，将解析后结构化的算法输出数据封装成ROS Msg后发布
  int PostProcess(const std::shared_ptr<hobot::dnn_node::DnnNodeOutput>&
                      node_output) override;

private:
  void RunLocalFeedInfer();
  void preprocessRun();

private:
  Model *model;
  std::string model_file_{"config/model/model.hbm"};
  std::string preprocess_config_file_{"config/centerpoint_preprocess_5dim.json"};
  std::string lidar_list_file_{"config/nuscenes_lidar_val.lst"};
  
  // 雷达文件数据路径
  // lidar_file_path = lidar_pre_path_ + (path in lidar_list_file_)
  std::string lidar_pre_path_ = "config/hobot_centerpoint_data";

  bool is_loop_{true}; // 是否循环回灌
  std::vector<std::string> local_file_list{}; // 回灌的本地二进制文件
  std::shared_ptr<std::thread> preprocess_thread_; // 预处理线程
  PCQueue<InputData> local_feed_queue; // 存放前处理后的回灌数据
  std::shared_ptr<PreProcess> preprocess_handle_{nullptr}; // 前处理对象指针

  std::shared_ptr<CenterPointPostProcess> postprocess_handle_{nullptr}; //后处理对象指针

  bool is_show_{true}; // 是否对结果进行渲染
  std::string pub_topic_name{"/hobot_centerpoint"}; // 结果发布的topic

  // 发布
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr centerpoint_pub_{nullptr};
  std::shared_ptr<Centerpoint_Publisher> sp_publisher{nullptr};
};

}  // namespace centerpoint
} // namespace hobot

#endif // INCLUDE_CENTERPOINT_NODE_H_
