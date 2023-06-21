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

#ifndef INCLUDE_CENTERPOINT_DATA_H
#define INCLUDE_CENTERPOINT_DATA_H

#include <algorithm>
#include <iomanip>
#include <iterator>
#include <ostream>
#include <sstream>
#include <string>
#include <utility>
#include <vector>
#include <memory>

namespace hobot {
namespace centerpoint {

typedef struct Lidar3D {
  float xs{0.0};
  float ys{0.0};
  float height{0.0};
  float dim_0{0.0};
  float dim_1{0.0};
  float dim_2{0.0};
  float rot{0.0};
  float vel_0{0.0};
  float vel_1{0.0};

  Lidar3D() {}

  Lidar3D(float x,
          float y,
          float h,
          float d_0,
          float d_1,
          float d_2,
          float r,
          float v_0,
          float v_1)
      : xs(x),
        ys(y),
        height(h),
        dim_0(d_0),
        dim_1(d_1),
        dim_2(d_2),
        rot(r),
        vel_0(v_0),
        vel_1(v_1) {}

  friend std::ostream &operator<<(std::ostream &os, const Lidar3D &bbox) {
    const auto precision = os.precision();
    const auto flags = os.flags();
    os << "[" << std::fixed << std::setprecision(6) << bbox.xs << "," << bbox.ys
       << "," << bbox.height << "," << bbox.dim_0 << "," << bbox.dim_1 << ","
       << bbox.dim_2 << "," << bbox.rot << "," << bbox.vel_0 << ","
       << bbox.vel_1 << "]";
    os.flags(flags);
    os.precision(precision);
    return os;
  }

  ~Lidar3D() {}
} Lidar3D;

typedef struct LidarDetection3D {
  Lidar3D bbox;
  float score{0.0};
  int label;  // classification lable
  LidarDetection3D() {}

  LidarDetection3D(Lidar3D bbox, float score, int label)
      : bbox(bbox), score(score), label(label) {}

  friend bool operator>(const LidarDetection3D &lhs,
                        const LidarDetection3D &rhs) {
    return (lhs.score > rhs.score);
  }

  friend std::ostream &operator<<(std::ostream &os,
                                  const LidarDetection3D &det) {
    const auto precision = os.precision();
    const auto flags = os.flags();
    os << "{"
       << R"("bbox")"
       << ":" << det.bbox << ","
       << R"("score")"
       << ":" << std::fixed << std::setprecision(6) << det.score << ","
       << R"("label")"
       << ":" << det.label << "\"}";
    os.flags(flags);
    os.precision(precision);
    return os;
  }
  ~LidarDetection3D() {}
} LidarDetection3D;

struct Perception {
  std::vector<LidarDetection3D> lidar3d;

  friend std::ostream &operator<<(std::ostream &os, Perception &perception) {
    os << "[";
    auto &rlts = perception.lidar3d;
      for (int i = 0; i < rlts.size(); i++) {
        if (i != 0) {
          os << ",";
        }
        os << rlts[i];
      }
    os << "]";
    return os;
  }
};

} // namespace centerpoint
} // namespace hobot

#endif // INCLUDE_CENTERPOINT_DATA_H
