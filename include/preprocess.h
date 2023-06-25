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

#ifndef INCLUDE_PREPROCESS_H_
#define INCLUDE_PREPROCESS_H_

#include <string>
#include <vector>
#include <memory>

#include "dnn_node/dnn_node.h"

namespace hobot {
namespace centerpoint {

using hobot::dnn_node::DNNInput;
using hobot::dnn_node::DNNResult;
using hobot::dnn_node::DNNTensor;
using hobot::dnn_node::Model;

struct VoxelConfig {
  const float kback_border;
  const float kfront_border;
  const float kright_border;
  const float kleft_border;
  const float kbottom_border;
  const float ktop_border;
  const float kr_lower;
  const float kr_upper;
  const float kx_scale;
  const float ky_scale;
  const int kmax_num_point_pillar;
  const int kmax_num_point;
  const int kdim;
  const int kx_length;
  const int ky_width;
  const float kx_range;
  const float ky_range;
  const float kz_range;
  const float kr_range;

  std::vector<int> kmax_num_point_pillar_vec = {
      0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19};

  int *pillar_point_num;
  int *coor_to_voxelidx;
  int *coors;

  VoxelConfig(float back,
              float front,
              float right,
              float left,
              float bottom,
              float top,
              float r_lower,
              float r_upper,
              float x_scale,
              float y_scale,
              int max_num_point_pillar,
              int max_num_point,
              int dim)
      : kback_border(back),
        kfront_border(front),
        kright_border(right),
        kleft_border(left),
        kbottom_border(bottom),
        ktop_border(top),
        kr_lower(r_lower),
        kr_upper(r_upper),
        kx_scale(x_scale),
        ky_scale(y_scale),
        kmax_num_point_pillar(max_num_point_pillar),
        kmax_num_point(max_num_point),
        kdim(dim),
        kx_length((kfront_border - kback_border) / kx_scale),
        ky_width((kleft_border - kright_border) / ky_scale),
        kx_range(kfront_border - kback_border),
        ky_range(kleft_border - kright_border),
        kz_range(ktop_border - kbottom_border),
        kr_range(kr_upper - kr_lower) {
    pillar_point_num = new int[this->kmax_num_point];
    coor_to_voxelidx = new int[kx_length * ky_width];
    coors = new int[this->kmax_num_point * 4];
  }

  ~VoxelConfig() {}
};

class PreProcess
{
public:
  explicit PreProcess(const std::string &config_file);
  int InitPreProcessInfo(const std::string &config_file);
  int DoProcess(std::string path,
                std::vector<std::shared_ptr<DNNTensor>> &input_tensors,
                Model *pmodel);
  ~PreProcess();
private:
  int32_t FetchBinary(std::string pointcloud_path,
                      float *buffer,
                      int32_t length);

  void GenVoxel(int start, int end);
  void GenFeatureDim5(float scale);
  void TransposeDim5();
  void Reset();

private:
  VoxelConfig *config_{nullptr};

  float *pointcloud_data_{nullptr};
  float *voxel_data_{nullptr};
  int8_t *features_s8_{nullptr};

  int voxel_num_{0};
};

}  // namespace centerpoint
} // namespace hobot

#endif // INCLUDE_PREPROCESS_H_