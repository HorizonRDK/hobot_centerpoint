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

#include "centerpoint_node.h"

namespace hobot {
namespace centerpoint {

CenterPoint_Node::CenterPoint_Node(const std::string& node_name,
                const rclcpp::NodeOptions& options)
      : hobot::dnn_node::DnnNode(node_name, options) {
  this->declare_parameter<std::string>("preprocess_config", preprocess_config_file_);
  this->declare_parameter<std::string>("model_file", model_file_);
  this->declare_parameter<std::string>("lidar_list_file", lidar_list_file_);
  this->declare_parameter<bool>("is_show", is_show_);
  this->declare_parameter<bool>("is_loop", is_loop_);
  this->declare_parameter<std::string>("pub_topic_name", pub_topic_name);
  this->declare_parameter<std::string>("lidar_pre_path", lidar_pre_path_);
  
  this->get_parameter<std::string>("preprocess_config", preprocess_config_file_);
  this->get_parameter<std::string>("model_file", model_file_);
  this->get_parameter<std::string>("lidar_list_file", lidar_list_file_);
  this->get_parameter<bool>("is_show", is_show_);
  this->get_parameter<bool>("is_loop", is_loop_);
  this->get_parameter<std::string>("pub_topic_name", pub_topic_name);
  this->get_parameter<std::string>("lidar_pre_path", lidar_pre_path_);

  RCLCPP_WARN_STREAM(rclcpp::get_logger("centerpoint_node"),
    "\n preprocess_config: " << preprocess_config_file_
    << "\n model_file: " << model_file_
    << "\n lidar_list_file: " << lidar_list_file_
    << "\n is_show: " << is_show_
    << "\n is_loop: " << is_loop_
    << "\n pub_topic_name: " << pub_topic_name
    << "\n lidar_pre_path: " << lidar_pre_path_);

  // Init中使用DNNNodeSample子类实现的SetNodePara()方法进行算法推理的初始化
  if (Init() != 0) {
    RCLCPP_ERROR(rclcpp::get_logger("centerpoint_node"), "Node init fail!");
    rclcpp::shutdown();
    return;
  }
  model = GetModel(); // 获取模型
  RunLocalFeedInfer(); // 调用本地回灌
}

int CenterPoint_Node::SetNodePara() {
  if (access(preprocess_config_file_.c_str(), F_OK) != 0) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("hobot_centerpoint"), "File is not exist! config_file_: " << preprocess_config_file_);
    return -1;
  }
  if (access(model_file_.c_str(), F_OK) != 0) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("hobot_centerpoint"), "File is not exist! model_file: " << model_file_);
    return -1;
  }

  dnn_node_para_ptr_->model_file = model_file_;
  dnn_node_para_ptr_->task_num = 4;

  preprocess_handle_ = std::make_shared<PreProcess>(preprocess_config_file_);
  postprocess_handle_ = std::make_shared<CenterPointPostProcess>();
  if(is_show_) { // 创建发布
    centerpoint_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
                                                    pub_topic_name, 10);
    sp_publisher = std::make_shared<Centerpoint_Publisher>(centerpoint_pub_);
  }
  return 0;
}

// 推理结果回调，解析算法输出
int CenterPoint_Node::PostProcess(
    const std::shared_ptr<hobot::dnn_node::DnnNodeOutput>& node_output) {
  if (!rclcpp::ok()) {
    return 0;
  }

  // 后处理开始时间
  auto tp_start = std::chrono::system_clock::now();
  // 开始解析
  std::shared_ptr<Perception> det_result = std::make_shared<Perception>();
  postprocess_handle_->OutputPostProcess(node_output, det_result);

  if (node_output->rt_stat) {
    auto tp_now = std::chrono::system_clock::now();
    auto interval = std::chrono::duration_cast<std::chrono::milliseconds>(
                        tp_now - tp_start)
                        .count();
    RCLCPP_WARN(rclcpp::get_logger("CenterPoint_Node"),
                "input fps: %.2f, out fps: %.2f, infer time ms: %d, "
                "post process time ms: %d",
                node_output->rt_stat->input_fps,
                node_output->rt_stat->output_fps,
                node_output->rt_stat->infer_time_ms,
                interval);
  }

  auto sp_centerpoint_node_out = std::dynamic_pointer_cast<CenterPointNodeOutput>(node_output);
  if (sp_centerpoint_node_out) {
    if (is_show_) { // 发布
      sp_publisher->publish(sp_centerpoint_node_out->lidar_files, det_result);
    }
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("CenterPoint_Node"),
                "Pointer cast fail");
  }
  return 0;
}

// 预处理线程函数
void CenterPoint_Node::preprocessRun() { 
  for (size_t i = 0; i < local_file_list.size(); i++) {
    if(!rclcpp::ok()) {
      return;
    }
    InputData data;
    data.lidar_files = local_file_list[i];
    // 输入预处理
    preprocess_handle_->DoProcess(data.lidar_files, data.input_tensors, model);
    local_feed_queue.put(data);
    // 循环读取
    if(is_loop_ && i + 1 == local_file_list.size()) {
      i = -1;
    }
  }
}

void CenterPoint_Node::RunLocalFeedInfer() {
  if (!model) {
    RCLCPP_ERROR(rclcpp::get_logger("hobot_centerpoint"), "Invalid model!");
    rclcpp::shutdown();
    return;
  }

  // 获取激光雷达本地输入文件
  if (access(lidar_list_file_.c_str(), F_OK) != 0) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("hobot_centerpoint"),
                      "File is not exist! lidar_list_file: " << lidar_list_file_);
      rclcpp::shutdown();
      return;
  }
  std::ifstream ifs(lidar_list_file_);
  std::string lidar_file;
  std::string file_path;
  while (std::getline(ifs, lidar_file)) {
    std::string file_path = lidar_pre_path_ + "/" + lidar_file;
    if (access(file_path.c_str(), F_OK) != 0) { // 二进制雷达文件不存在，跳过
      RCLCPP_ERROR_STREAM(rclcpp::get_logger("hobot_centerpoint"),
                      "File is not exist! lidar_file: " << file_path);
      continue;
    }
    local_file_list.push_back(file_path);
  }
  RCLCPP_WARN(rclcpp::get_logger("hobot_centerpoint"), 
              "A total of %d files were fetched! ", local_file_list.size());

  // 创建预处理线程
  preprocess_thread_ = std::make_shared<std::thread>(&CenterPoint_Node::preprocessRun, this);

  while (rclcpp::ok()) {
    InputData input;
    if (!local_feed_queue.get(input, 1000)) {
      continue;
    }

    std::vector<std::shared_ptr<hobot::dnn_node::OutputDescription>> output_descs{};
    auto dnn_output = std::make_shared<CenterPointNodeOutput>();
    dnn_output->lidar_files = input.lidar_files;

    // 模型推理开始
    if (Run(input.input_tensors, output_descs, dnn_output, true, -1, -1) < 0) {
      RCLCPP_INFO(rclcpp::get_logger("hobot_centerpoint"), "Run infer fail!");
    }

    // 释放input_tensors
    for (size_t i = 0; i < input.input_tensors.size(); i++) {
      hbSysFreeMem(&(input.input_tensors[i]->sysMem[0]));
    }
  }
}

} // namespace centerpoint
} // namespace hobot
