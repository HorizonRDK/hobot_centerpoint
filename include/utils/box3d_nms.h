// Copyright (c) 2020 Horizon Robotics.All Rights Reserved.
//
// The material in this file is confidential and contains trade secrets
// of Horizon Robotics Inc. This is proprietary information owned by
// Horizon Robotics Inc. No part of this work may be disclosed,
// reproduced, copied, transmitted, or used in any way for any purpose,
// without the express written permission of Horizon Robotics Inc.

#ifndef _UTILS_BOX3D_NMS_H_
#define _UTILS_BOX3D_NMS_H_

#include <vector>

struct RotatedBox {
  float x_ctr;
  float y_ctr;
  float w;
  float h;
  float a;
};

void Lidar3DNMS(std::vector<int> &keep_res,
                std::vector<std::vector<float>> &boxes,
                std::vector<float> &scores,
                float thresh,
                int per_max_size,
                int post_max_size);

void Lidar3DNmsRotated(std::vector<int> &keep,
                       std::vector<std::vector<float>> &dets,
                       std::vector<float> &scores,
                       float iou_threshold);

float single_box_iou_rotated(std::vector<float> &box1_raw,
                             std::vector<float> &box2_raw,
                             int mode_flag);

#endif  // _UTILS_BOX3D_NMS_H_
