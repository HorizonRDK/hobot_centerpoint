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

#include "jsonutil.h"
#include "dnn_node/dnn_node.h"
#include "dnn_node/util/image_proc.h"
#include "preprocess.h"
#include "postprocess.h"
#include "centerpoint_publisher.h"


namespace hobot {
namespace centerpoint {

struct CenterPointNodeOutput : public hobot::dnn_node::DnnNodeOutput {
  std::string lidar_files;
};

enum class ShowType {
  None = 0,
  WEB_NODE = 1,
  FOXGLOVE = 2
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

private:
  std::string model_file_ = "config/model/model.hbm";
  std::string preprocess_config_file_ = "config/centerpoint_preprocess_5dim.json";
  std::string lidar_list_file_ = "config/nuscenes_lidar/nuscenes_lidar.lst";

  bool is_loop_ = true;
  // int feed_type_ = 0;
  std::vector<std::string> local_file_list;

  std::shared_ptr<PreProcess> preprocess_handle_ = nullptr;
  std::shared_ptr<CenterPointPostProcess> postprocess_handle_ = nullptr;

  // ShowType show_type = ShowType::WEB_NODE;
  bool is_show_ = true;
  std::string pub_topic_name = "/hobot_centerpoint";
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr centerpoint_pub_ = nullptr;
  std::shared_ptr<Centerpoint_Publisher> sp_publisher = nullptr;
};

}  // namespace centerpoint
} // namespace hobot

#endif // INCLUDE_CENTERPOINT_NODE_H_
