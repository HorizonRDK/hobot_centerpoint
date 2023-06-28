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

#ifndef INCLUDE_CENTERPOINT_PUBLISHER_H
#define INCLUDE_CENTERPOINT_PUBLISHER_H

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include "sensor_msgs/msg/image.hpp"
#include "opencv2/core/mat.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/core/types.hpp"

#include "centerpoint_data.h"

namespace hobot {
namespace centerpoint {

class Centerpoint_Publisher
{
public:
  Centerpoint_Publisher(rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr sp_pub);
  int publish(std::string &pointclude_file, std::shared_ptr<Perception> &perception);

private:
  int LoadLidarFile(std::string &pointclude_file, cv::Mat &image,
                      float &width_offset, float &height_offset,
                      int &width_resize, int &height_resize,
                      int &width, int &height);
  int Draw_perception(std::shared_ptr<Perception> &perception, cv::Mat &image, 
                      float &width_offset, float &height_offset,
                      int &width_resize, int &height_resize,
                      int &width, int &height);

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr centerpoint_pub_ = nullptr;

  // 自车在雷达图中的点位坐标, 输入的点云图不同，自车坐标会变化
  int selfCar_point_x = 0.0;
  int selfCar_point_y = 0.0;
};

} // namespace centerpoint
} // namespace hobot

#endif // INCLUDE_CENTERPOINT_PUBLISHER_H
