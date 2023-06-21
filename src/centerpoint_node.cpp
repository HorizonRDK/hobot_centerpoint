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
  // this->declare_parameter<int>("feed_type", feed_type_);
  this->declare_parameter<std::string>("pub_topic_name", pub_topic_name);
  
  this->get_parameter<std::string>("preprocess_config", preprocess_config_file_);
  this->get_parameter<std::string>("model_file", model_file_);
  this->get_parameter<std::string>("lidar_list_file", lidar_list_file_);
  this->get_parameter<bool>("is_show", is_show_);
  this->get_parameter<bool>("is_loop", is_loop_);
  // this->get_parameter<int>("feed_type", feed_type_);
  this->get_parameter<std::string>("pub_topic_name", pub_topic_name);

  RCLCPP_WARN_STREAM(rclcpp::get_logger("centerpoint_node"),
    "\n preprocess_config: " << preprocess_config_file_
    << "\n model_file: " << model_file_
    << "\n lidar_list_file: " << lidar_list_file_
    << "\n is_show: " << is_show_
    << "\n is_loop: " << is_loop_
    // << "\n feed_type: " << feed_type_
    << "\n pub_topic_name" << pub_topic_name);

  // Init中使用DNNNodeSample子类实现的SetNodePara()方法进行算法推理的初始化
  if (Init() != 0) {
    RCLCPP_ERROR(rclcpp::get_logger("centerpoint_node"), "Node init fail!");
    rclcpp::shutdown();
    return;
  }

  RunLocalFeedInfer();
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
  if(is_show_) {
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
  RCLCPP_WARN(rclcpp::get_logger("CenterPoint_Node"), "post process start!");
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
    if (is_show_) {
      auto pub_start = std::chrono::system_clock::now();
      RCLCPP_WARN(rclcpp::get_logger("CenterPoint_Node"), "begin publish result!");
      sp_publisher->publish(sp_centerpoint_node_out->lidar_files, det_result);
      auto tp_now = std::chrono::system_clock::now();
      auto interval = std::chrono::duration_cast<std::chrono::milliseconds>(
                        tp_now - pub_start)
                        .count();
      RCLCPP_WARN(rclcpp::get_logger("CenterPoint_Node"), " publish over! time ms: %d", interval);
      // std::stringstream result_log;
      // result_log << std::setprecision(10) << sp_centerpoint_node_out->lidar_files << ":";
      // for (auto &rst : det_result->lidar3d) {
      //   if (rst.score < 4.0) {
      //     continue;
      //   }
      //   result_log << rst.label << " " << rst.score << " " << rst.bbox.xs << " "
      //             << rst.bbox.ys << " " << rst.bbox.height << " " << rst.bbox.dim_0
      //             << " " << rst.bbox.dim_1 << " " << rst.bbox.dim_2 << " "
      //             << rst.bbox.rot << " " << rst.bbox.vel_0 << " " << rst.bbox.vel_1
      //             << "; ";
      // }
    }
  }
}

void CenterPoint_Node::RunLocalFeedInfer() {
  auto model = GetModel();
  if (!model) {
    RCLCPP_ERROR(rclcpp::get_logger("hobot_centerpoint"), "Invalid model!");
    rclcpp::shutdown();
    return;
  }

  // 获取激光雷达本地输入文件
  std::ifstream ifs(lidar_list_file_);
  std::string lidar_file;
  while (std::getline(ifs, lidar_file)) {
    if (access(lidar_file.c_str(), F_OK) != 0) {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("hobot_centerpoint"),
                        "File is not exist! lidar_file: " << lidar_file);
        rclcpp::shutdown();
        return;
      }
      local_file_list.push_back(lidar_file);
  }
  
  for (size_t i = 0; i < local_file_list.size(); i++) {
    std::vector<std::shared_ptr<DNNTensor>> input_tensors;
    // 输入预处理
    preprocess_handle_->DoProcess(local_file_list[i], input_tensors, model);

    std::vector<std::shared_ptr<hobot::dnn_node::OutputDescription>> output_descs{};
    auto dnn_output = std::make_shared<CenterPointNodeOutput>();
    dnn_output->lidar_files = local_file_list[i];

    // 模型推理开始
    if (Run(input_tensors, output_descs, dnn_output, true, -1, -1) < 0) {
      RCLCPP_INFO(rclcpp::get_logger("hobot_centerpoint"), "Run infer fail!");
    }

    // 循环读取
    if(is_loop_ && i + 1 == local_file_list.size()) {
      i = 0;
    }
  }
  
}

} // namespace centerpoint
} // namespace hobot
