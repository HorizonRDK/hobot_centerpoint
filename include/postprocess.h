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

#ifndef INCLUDE_POSTPROCESS_H_
#define INCLUDE_POSTPROCESS_H_

#include "jsonutil.h"
#include "centerpoint_data.h"
#include "dnn_node/dnn_node.h"

#include <memory>

namespace hobot {
namespace centerpoint {

using hobot::dnn_node::DNNTensor;

struct ScoresData {
  float value;
  int c;
  int w;
  int h;

  ScoresData(float score, int channel, float x, float y)
      : value(score), c(channel), w(x), h(y) {}

  friend std::ostream &operator<<(std::ostream &os, const ScoresData &scores) {
    os << std::fixed << std::setprecision(6) << scores.value << ", " << scores.c
       << ", " << scores.h << ", " << scores.w << std::endl;
    return os;
  }

  friend bool operator>(const ScoresData &lhs, const ScoresData &rhs) {
    return (lhs.value >= rhs.value);
  }
};

struct CompareScoresData {
  bool operator()(const ScoresData &lhs, const ScoresData &rhs) const {
    return lhs.value >= rhs.value;
  }
};

class CenterPointPostProcess {
  public:
    explicit CenterPointPostProcess() {

    }

    int OutputPostProcess(const std::shared_ptr<hobot::dnn_node::DnnNodeOutput>& node_output,
                            std::shared_ptr<Perception>& result);

  private:
    // int InitPostProcess(const std::string &config_file);
    void HeatmapDequantizeScale(int *valid_shape,
                            int *aligned_shape,
                            float *scale,
                            int topk,
                            int32_t *input,
                            std::vector<ScoresData> &output);
    std::vector<std::vector<float>> xywhr2xyxyr(std::vector<Lidar3D> &Bboxes);

  private:
    bool norm_bbox_{true};
    int topk_{500};
    int out_size_factor_{4};
    float score_threshold_{0.1};
    int pre_max_size_{1000};
    int post_max_size_{83};
    float nms_thr_{0.2};
    std::vector<float> pc_range_{-51.2, -51.2};
    std::vector<float> voxel_size_{0.2, 0.2};
    std::vector<float> post_center_range_{-61.2, -61.2, -10.0, 61.2, 61.2, 10.0};

    int height_{128};
    int width_{128};
    int aligned_c_{4};
};

}  // namespace centerpoint
} // namespace hobot

#endif // INCLUDE_POSTPROCESS_H_
