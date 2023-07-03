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

#include "preprocess.h"
#include "jsonutil.h"

#include <limits.h>
#include <math.h>
#include <cfenv>
#include <chrono>
#include <ctime>
#include <iostream>

namespace hobot {
namespace centerpoint {

static inline float round(float const input) {
  std::fesetround(FE_TONEAREST);
  float const result{std::nearbyintf(input)};
  return result;
}

PreProcess::PreProcess(const std::string &config_file) {
  InitPreProcessInfo(config_file);
}

int PreProcess::InitPreProcessInfo(const std::string &config) {
  RCLCPP_INFO(rclcpp::get_logger("hobot_centerpoint"), 
              "Preprocess init config: %s", config.c_str());

  rapidjson::Document document;
  if (JSONUtil::ParseJson(config, document) < 0) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("hobot_centerpoint"),
                        "Preprocess parsing fail! config_file: " << config);
    return -1;
  }
  // document.Parse(config.data());

  if (document.HasParseError()) {
    RCLCPP_INFO(rclcpp::get_logger("hobot_centerpoint"),
                "Preprocess parsing config file failed");
    return -1;
  }

  float back = document["back"].GetFloat();

  float front = document["front"].GetFloat();

  float right = document["right"].GetFloat();

  float left = document["left"].GetFloat();

  float bottom = document["bottom"].GetFloat();

  float top = document["top"].GetFloat();

  float r_lower = document["r_lower"].GetFloat();

  float r_upper = document["r_upper"].GetFloat();

  float x_scale = document["x_scale"].GetFloat();

  float y_scale = document["y_scale"].GetFloat();

  int32_t max_num_point_pillar = document["max_num_point_pillar"].GetInt();

  int32_t max_num_point = document["max_num_point"].GetInt();

  int32_t dim = document["dim"].GetInt();

  config_ = new VoxelConfig(back,
                            front,
                            right,
                            left,
                            bottom,
                            top,
                            r_lower,
                            r_upper,
                            x_scale,
                            y_scale,
                            max_num_point_pillar,
                            max_num_point,
                            dim);

  pointcloud_data_ =
      static_cast<float *>(malloc(300000 * config_->kdim * sizeof(float)));
  voxel_data_ = static_cast<float *>(
      malloc(config_->kmax_num_point * config_->kmax_num_point_pillar *
             config_->kdim * sizeof(float)));
  features_s8_ = static_cast<int8_t *>(
      malloc(config_->kmax_num_point * config_->kmax_num_point_pillar *
             config_->kdim * sizeof(int8_t)));

  std::stringstream ss;
  ss << "VoxelConfig: { "
          "kback_border: "
      << config_->kback_border
      << ", kfront_border: " << config_->kfront_border
      << ", kright_border: " << config_->kright_border
      << ", kleft_border: " << config_->kleft_border
      << ", kbottom_border: " << config_->kbottom_border
      << ", ktop_border: " << config_->ktop_border
      << ", kr_lower: " << config_->kr_lower
      << ", kr_upper: " << config_->kr_upper
      << ", kx_range: " << config_->kx_range
      << ", ky_range: " << config_->ky_range
      << ", kz_range: " << config_->kz_range
      << ", kr_range: " << config_->kr_range
      << ", kx_scale: " << config_->kx_scale
      << ", ky_scale: " << config_->ky_scale
      << ", kx_length: " << config_->kx_length
      << ", ky_width: " << config_->ky_width
      << ", kmax_num_point_pillar: "
      << config_->kmax_num_point_pillar
      << ", kmax_num_point: " << config_->kmax_num_point
      << ", kdim: " << config_->kdim << " }";

  RCLCPP_INFO(rclcpp::get_logger("hobot_centerpoint"), "%s", ss.str().c_str());

  return 0;
}

int32_t PreProcess::FetchBinary(std::string pointcloud_path,
                                                    float *buffer,
                                                    int32_t length) {
  FILE *stream = nullptr;
  if (pointcloud_path.empty()) {
    RCLCPP_ERROR(rclcpp::get_logger("hobot_centerpoint"),
                "bin file: %s is empty!", pointcloud_path.c_str());
    return -1;
  }

  stream = fopen(pointcloud_path.c_str(), "rb");
  if (!stream) {
    RCLCPP_ERROR(rclcpp::get_logger("hobot_centerpoint"),
                "bin file: %s open failed!", pointcloud_path.c_str());
    return -1;
  }

  size_t ret_code = fread(buffer, sizeof(float), length, stream);
  int32_t num = ret_code / config_->kdim;
  fclose(stream);
  return num;
}

void PreProcess::Reset() {
  // Reset # of points in each pillar.
  memset(config_->pillar_point_num, 0, config_->kmax_num_point * sizeof(int));

  memset(config_->coor_to_voxelidx,
         -1,
         config_->kx_length * config_->ky_width * sizeof(int));

  memset(config_->coors, -1, config_->kmax_num_point * 4 * sizeof(int));
  for (int32_t i = 0; i < config_->kmax_num_point; i++) {
    config_->coors[i * 4 + 0] = 0;
  }

  memset(voxel_data_,
         0,
         config_->kmax_num_point * config_->kmax_num_point_pillar *
             config_->kdim * sizeof(float));

  memset(features_s8_,
         0,
         config_->kmax_num_point * config_->kmax_num_point_pillar *
             config_->kdim * sizeof(int8_t));

  voxel_num_ = 0;
}

void PreProcess::GenVoxel(int start, int end) {
  for (int i = start; i < end; i++) {
    float *point;
    point = pointcloud_data_ + i * config_->kdim;

    float &point_x = point[0];
    float &point_y = point[1];
    float &point_z = point[2];
    float &point_r = point[3];

    if (point_x <= config_->kback_border || point_x >= config_->kfront_border ||
        point_y <= config_->kright_border || point_y >= config_->kleft_border ||
        point_z <= config_->kbottom_border || point_z >= config_->ktop_border) {
      continue;
    }

    int idx = (point_x - config_->kback_border) / config_->kx_scale;
    int idy = (point_y - config_->kright_border) / config_->ky_scale;

    // zyx
    int pillar_index = idy * config_->kx_length + idx;
    int voxelidx = config_->coor_to_voxelidx[pillar_index];
    if (voxelidx == -1) {
      if (voxel_num_ < config_->kmax_num_point) {
        voxelidx = voxel_num_;
        voxel_num_ += 1;
      } else {
        voxelidx = config_->kmax_num_point - 1;
      }
      config_->coor_to_voxelidx[pillar_index] = voxelidx;
      config_->coors[voxelidx * 4 + 1] = 0;
      config_->coors[voxelidx * 4 + 2] = idy;
      config_->coors[voxelidx * 4 + 3] = idx;
    }

    if (config_->pillar_point_num[voxelidx] >= config_->kmax_num_point_pillar) {
      continue;
    } else {
      auto total_offset = (config_->kmax_num_point_pillar * voxelidx +
                           config_->pillar_point_num[voxelidx]) *
                          config_->kdim;
      config_->pillar_point_num[voxelidx] += 1;

      *(voxel_data_ + total_offset) = point_x;
      *(voxel_data_ + total_offset + 1) = point_y;
      *(voxel_data_ + total_offset + 2) = point_z;
      *(voxel_data_ + total_offset + 3) = point_r;
      *(voxel_data_ + total_offset + 4) = point[4];
    }
  }
}

void PreProcess::GenFeatureDim5(float scale) {
  for (int i = 0; i < voxel_num_; i++) {
    int idx = i * config_->kmax_num_point_pillar * config_->kdim;
    for (int j = 0; j < config_->kmax_num_point_pillar; ++j) {
      if (config_->pillar_point_num[i] >
          config_->kmax_num_point_pillar_vec[j]) {
        int index = idx + j * config_->kdim;
        voxel_data_[index + 0] =
            (voxel_data_[index + 0] - config_->kback_border) /
            config_->kx_range / scale;
        voxel_data_[index + 1] =
            (voxel_data_[index + 1] - config_->kright_border) /
            config_->ky_range / scale;
        voxel_data_[index + 2] =
            (voxel_data_[index + 2] - config_->kbottom_border) /
            config_->kz_range / scale;
        voxel_data_[index + 3] = (voxel_data_[index + 3] - config_->kr_lower) /
                                 config_->kr_range / scale;
        if (voxel_data_[index + 4] != 0) {
          voxel_data_[index + 4] = voxel_data_[index + 4] / scale;
        }
      }
    }
  }
}

void PreProcess::TransposeDim5() {
  // Transpose to 1x5x20x40000
  int kWC = config_->kmax_num_point_pillar * config_->kdim;
  int kHW = config_->kmax_num_point * config_->kmax_num_point_pillar;
  for (int c = 0; c < config_->kdim; ++c) {
    for (int w = 0; w < config_->kmax_num_point_pillar; ++w) {
      for (int h = 0; h < voxel_num_; ++h) {
        int old_index = h * kWC + w * config_->kdim + c;
        int new_index = c * kHW + w * config_->kmax_num_point + h;
        float features_tmp = round(static_cast<float>(voxel_data_[old_index]));
        features_tmp = std::min(std::max(features_tmp, -128.f), 127.f);
        features_s8_[new_index] = static_cast<int8_t>(features_tmp);
      }
    }
  }
}

int32_t PreProcess::DoProcess(std::string path,
                              std::vector<std::shared_ptr<DNNTensor>> &input_tensors,
                              Model *pmodel) {
  Reset();
  int32_t num_points =
      FetchBinary(path, pointcloud_data_, 300000 * config_->kdim);
  if (num_points == -1) {
    RCLCPP_ERROR(rclcpp::get_logger("hobot_centerpoint"), "Read file: %s error.", path.c_str());
    return -1;
  }

  auto input_count = pmodel->GetInputCount();

  for (int i = 0; i < input_count; i++) {
    auto input = std::make_shared<DNNTensor>();
    hbDNNTensorProperties properties;
    pmodel->GetInputTensorProperties(properties, i);
    int aligned_size = properties.alignedByteSize;
    hbSysAllocCachedMem(&(input->sysMem[0]), aligned_size);
    properties.alignedShape = properties.validShape;
    input->properties = properties;
    input_tensors.emplace_back(input);
  }

  // gen voxel
  int32_t *coors = reinterpret_cast<int32_t *>(input_tensors[0]->sysMem[0].virAddr);
  GenVoxel(0, num_points);
  memcpy(coors, config_->coors, input_tensors[0]->sysMem[0].memSize);

  // gen feature
  float scale_data = input_tensors[1]->properties.scale.scaleData[0];
  GenFeatureDim5(scale_data);

  // transpose
  int8_t *features = reinterpret_cast<int8_t *>(input_tensors[1]->sysMem[0].virAddr);
  TransposeDim5();
  memcpy(features, features_s8_, input_tensors[1]->sysMem[0].memSize);

  for (int32_t i = 0; i < input_count; i++) {
    hbSysFlushMem(&(input_tensors[i]->sysMem[0]), HB_SYS_MEM_CACHE_CLEAN);
  }

  return 0;
}

PreProcess::~PreProcess() {
  delete config_;
  config_ = nullptr;
  free(voxel_data_);
  free(pointcloud_data_);
  free(features_s8_);
}


}  // namespace centerpoint
} // namespace hobot
