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

#include "postprocess.h"
#include "utils/box3d_nms.h"

#include <queue>
#include <cmath>

namespace hobot {
namespace centerpoint {

float quanti_scale(int32_t data, float scale) { return data * scale; }

void CenterPointPostProcess::HeatmapDequantizeScale(int *valid_shape,
                            int *aligned_shape,
                            float *scale,
                            int topk,
                            int32_t *input,
                            std::vector<ScoresData> &output) {
  int kWC = aligned_shape[2] * aligned_shape[3];
  int kHW = valid_shape[1] * valid_shape[2];
  for (int c = 0; c < valid_shape[3]; ++c) {
    std::priority_queue<ScoresData,
                        std::vector<ScoresData>,
                        std::greater<ScoresData>>
        queue;
    for (int h = 0; h < valid_shape[1]; ++h) {
      for (int w = 0; w < valid_shape[2]; w++) {
        float data_tmp =
            quanti_scale(input[h * kWC + w * aligned_shape[3] + c], scale[c]);
        float value = 1.0f / (std::exp(-data_tmp) + 1.0f);
        queue.push(ScoresData(value, c, w, h));
        if (queue.size() > topk) {
          queue.pop();
        }
      }
    }
    while (!queue.empty()) {
      output.emplace_back(queue.top());
      queue.pop();
    }
  }
}

std::vector<std::vector<float>> CenterPointPostProcess::xywhr2xyxyr(std::vector<Lidar3D> &Bboxes) {
  std::vector<std::vector<float>> boxes(Bboxes.size(), std::vector<float>(5));
  for (int i = 0; i < Bboxes.size(); i++) {
    boxes[i][0] = Bboxes[i].xs - Bboxes[i].dim_0 * 0.5f;
    boxes[i][1] = Bboxes[i].ys - Bboxes[i].dim_1 * 0.5f;
    boxes[i][2] = Bboxes[i].xs + Bboxes[i].dim_0 * 0.5f;
    boxes[i][3] = Bboxes[i].ys + Bboxes[i].dim_1 * 0.5f;
    boxes[i][4] = Bboxes[i].rot;
  }
  return boxes;
}

// CenterPointPostProcess::CenterPointPostProcess(const std::string &config_file) {
//   InitPostProcess(config_file);
// }

// int CenterPointPostProcess::InitPostProcess(const std::string &config) {
//   RCLCPP_INFO(rclcpp::get_logger("hobot_centerpoint"),
//               "Init postProcess from Json: %s", config.c_str());

//   rapidjson::Document document;
//   document.Parse(config.data());

//   if (document.HasParseError()) {
//     RCLCPP_ERROR(rclcpp::get_logger("hobot_centerpoint"),
//                   "Parsing postprocess config file failed");
//     return -1;
//   }

//   if (document.HasMember("norm_bbox")) {
//     norm_bbox_ = document["norm_bbox"].GetBool();
//   }

//   if (document.HasMember("topk")) {
//     topk_ = document["topk"].GetInt();
//   }

//   if (document.HasMember("out_size_factor")) {
//     out_size_factor_ = document["out_size_factor"].GetInt();
//   }

//   if (document.HasMember("score_threshold")) {
//     score_threshold_ = document["score_threshold"].GetFloat();
//   }

//   if (document.HasMember("nms_thr")) {
//     nms_thr_ = document["nms_thr"].GetFloat();
//   }

//   if (document.HasMember("pre_max_size")) {
//     pre_max_size_ = document["pre_max_size"].GetInt();
//   }

//   if (document.HasMember("post_max_size")) {
//     post_max_size_ = document["post_max_size"].GetInt();
//   }

//   if (document.HasMember("pc_range")) {
//     auto pc_range_value = document["pc_range"].GetArray();
//     pc_range_.resize(pc_range_value.Size());
//     for (int i = 0; i < pc_range_value.Size(); i++) {
//       pc_range_[i] = pc_range_value[i].GetFloat();
//     }
//   }

//   if (document.HasMember("voxel_size")) {
//     auto voxel_size_value = document["voxel_size"].GetArray();
//     voxel_size_.resize(voxel_size_value.Size());
//     for (int i = 0; i < voxel_size_value.Size(); i++) {
//       voxel_size_[i] = voxel_size_value[i].GetFloat();
//     }
//   }

//   if (document.HasMember("post_center_range")) {
//     auto post_center_range_value = document["post_center_range"].GetArray();
//     post_center_range_.resize(post_center_range_value.Size());
//     for (int i = 0; i < post_center_range_value.Size(); i++) {
//       post_center_range_[i] = post_center_range_value[i].GetFloat();
//     }
//   }

//   return 0;
// }

int CenterPointPostProcess::OutputPostProcess(
                            const std::shared_ptr<hobot::dnn_node::DnnNodeOutput>& node_output,
                            std::shared_ptr<Perception>& result) {
  std::vector<std::shared_ptr<DNNTensor>> &tensors = node_output->output_tensors;
  for (int i = 0; i < tensors.size(); i++) {
    hbSysFlushMem(&(tensors[i]->sysMem[0]), HB_SYS_MEM_CACHE_INVALIDATE);
  }

  int kHW = width_ * height_;
  int aligned_WC = width_ * aligned_c_;
  int task_cls{0};

  for (int i = 0; i < 6; i++) {
    // heatmap: score, cls, id
    std::vector<ScoresData> heatmaps;
    int32_t *heatmap_data =
        reinterpret_cast<int32_t *>(tensors[i * 6 + 5]->sysMem->virAddr);
    float *heatmap_scale = tensors[i * 6 + 5]->properties.scale.scaleData;
    int32_t *heatmap_shape =
        tensors[i * 6 + 5]->properties.validShape.dimensionSize;
    int32_t *heatmap_aligned_shape =
        tensors[i * 6 + 5]->properties.alignedShape.dimensionSize;
    int32_t num_cls = heatmap_shape[3];
    HeatmapDequantizeScale(heatmap_shape,
                           heatmap_aligned_shape,
                           heatmap_scale,
                           topk_,
                           heatmap_data,
                           heatmaps);

    std::vector<ScoresData> heatmap;
    std::stable_sort(heatmaps.begin(), heatmaps.end(), CompareScoresData());
    for (auto &heat : heatmaps) {
      if (heat.value > score_threshold_) {
        heatmap.emplace_back(heat);
      }
    }
    int32_t score_size = heatmap.size();

    // rot
    int32_t *rot_data =
        reinterpret_cast<int32_t *>(tensors[i * 6 + 3]->sysMem->virAddr);
    float *rot_scale = tensors[i * 6 + 3]->properties.scale.scaleData;
    // height
    int32_t *height_data =
        reinterpret_cast<int32_t *>(tensors[i * 6 + 1]->sysMem->virAddr);
    float *height_scale = tensors[i * 6 + 1]->properties.scale.scaleData;
    // dim
    int32_t *dim_data =
        reinterpret_cast<int32_t *>(tensors[i * 6 + 2]->sysMem->virAddr);
    float *dim_scale = tensors[i * 6 + 2]->properties.scale.scaleData;
    // vel
    int32_t *vel_data =
        reinterpret_cast<int32_t *>(tensors[i * 6 + 4]->sysMem->virAddr);
    float *vel_scale = tensors[i * 6 + 4]->properties.scale.scaleData;
    // reg
    int32_t *reg_data =
        reinterpret_cast<int32_t *>(tensors[i * 6 + 0]->sysMem->virAddr);
    float *reg_scale = tensors[i * 6 + 0]->properties.scale.scaleData;

    std::vector<float> height;
    std::vector<float> xs;
    std::vector<float> ys;
    float xs_weight = out_size_factor_ * voxel_size_[0];
    float ys_weight = out_size_factor_ * voxel_size_[1];
    for (int k = 0; k < score_size; k++) {
      int32_t w = heatmap[k].w;
      int32_t h = heatmap[k].h;
      int32_t idx_0 = h * aligned_WC + w * aligned_c_ + 0;
      int32_t idx_1 = h * aligned_WC + w * aligned_c_ + 1;

      float reg_0 = quanti_scale(reg_data[idx_0], reg_scale[0]);
      float reg_1 = quanti_scale(reg_data[idx_1], reg_scale[1]);
      height.emplace_back(quanti_scale(height_data[idx_0], height_scale[0]));
      xs.emplace_back((reg_0 + heatmap[k].w) * xs_weight + pc_range_[0]);
      ys.emplace_back((reg_1 + heatmap[k].h) * ys_weight + pc_range_[1]);
    }

    // mask
    std::vector<int32_t> indexes;
    std::vector<Lidar3D> Bbox;
    std::vector<float> scores;
    std::vector<int32_t> cls;
    for (int32_t k = 0; k < score_size; k++) {
      if (xs[k] >= post_center_range_[0] && ys[k] >= post_center_range_[1] &&
          height[k] >= post_center_range_[2] &&
          xs[k] <= post_center_range_[3] && ys[k] <= post_center_range_[4] &&
          height[k] <= post_center_range_[5]) {
        indexes.emplace_back(k);
      }
    }

    Bbox.resize(indexes.size());
    scores.resize(indexes.size());
    cls.resize(indexes.size());

    for (int32_t j = 0; j < indexes.size(); j++) {
      int32_t k = indexes[j];
      int32_t w = heatmap[k].w;
      int32_t h = heatmap[k].h;
      int32_t idx_0 = h * aligned_WC + w * aligned_c_ + 0;
      int32_t idx_1 = h * aligned_WC + w * aligned_c_ + 1;
      int32_t idx_2 = h * aligned_WC + w * aligned_c_ + 2;
      float rots = quanti_scale(rot_data[idx_0], rot_scale[0]);
      float rotc = quanti_scale(rot_data[idx_1], rot_scale[1]);

      Bbox[j].xs = xs[k];
      Bbox[j].ys = ys[k];
      Bbox[j].height = height[k];

      float dim_0_tmp = quanti_scale(dim_data[idx_0], dim_scale[0]);
      float dim_1_tmp = quanti_scale(dim_data[idx_1], dim_scale[1]);
      float dim_2_tmp = quanti_scale(dim_data[idx_2], dim_scale[2]);
      Bbox[j].dim_0 = norm_bbox_ ? std::exp(dim_0_tmp) : dim_0_tmp;
      Bbox[j].dim_1 = norm_bbox_ ? std::exp(dim_1_tmp) : dim_1_tmp;
      Bbox[j].dim_2 = norm_bbox_ ? std::exp(dim_2_tmp) : dim_2_tmp;

      Bbox[j].rot = std::atan2(rots, rotc);
      Bbox[j].vel_0 = quanti_scale(vel_data[idx_0], vel_scale[0]);
      Bbox[j].vel_1 = quanti_scale(vel_data[idx_1], vel_scale[1]);

      scores[j] = heatmap[k].value;
      cls[j] = heatmap[k].c;
    }

    std::vector<int32_t> selected_idxs;

    if (Bbox.size() > 0) {
      std::vector<std::vector<float>> bboxes_for_nms = xywhr2xyxyr(Bbox);
      Lidar3DNMS(selected_idxs,
                 bboxes_for_nms,
                 scores,
                 nms_thr_,
                 pre_max_size_,
                 post_max_size_);

      if (selected_idxs.size() > 0) {
        for (int32_t s = 0; s < selected_idxs.size(); s++) {
          int k = selected_idxs[s];
          Bbox[k].height = Bbox[k].height - Bbox[k].dim_2 * 0.5f;
          result->lidar3d.emplace_back(
              LidarDetection3D{Bbox[k], scores[k], cls[k] + task_cls});
        }
      }
    }
    task_cls += num_cls;
  }

  if (result->lidar3d.size() == 0) {
    result->lidar3d.resize(6, LidarDetection3D{Lidar3D(), 0.0f, 0});
  }
  return 0;
}

}  // namespace centerpoint
} // namespace hobot
