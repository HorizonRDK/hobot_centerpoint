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

#include "centerpoint_publisher.h"
#include <fstream>
#include <iostream>

#define PUBIMAGE_WIDTH 1920
#define PUBIMAGE_HEIGHT 1440

// 读取二进制点云文件
int read_binary_file(std::string &file_path, char **bin, int *length) {
  std::ifstream ifs(file_path.c_str(), std::ios::in | std::ios::binary);
  if (!ifs) {
    std::cout << "Can't open files: "<< file_path<<std::endl;
    return -1;
  }
  ifs.seekg(0, std::ios::end);
  *length = ifs.tellg();
  ifs.seekg(0, std::ios::beg);
  *bin = new char[sizeof(char) * (*length)];
  ifs.read(*bin, *length);
  ifs.close();
  return 0;
}

// (i, j) = (i, a) x (a, j)
int matrix_mul(std::vector<std::vector<float>> &A,
                      std::vector<std::vector<float>> &B,
                      std::vector<std::vector<float>> &C) {
  if (A[0].size() != B.size()) {
    RCLCPP_ERROR(rclcpp::get_logger("hobot_centerpoint_pub"), "matrix_mul shapes mismatch...");
    return -1;
  }

  int height = C.size();
  int width = C[0].size();
  int common_length = B.size();

  for (int i = 0; i < height; ++i) {
    for (int j = 0; j < width; ++j) {
      for (int c = 0; c < common_length; ++c) {
        C[i][j] += A[i][c] * B[c][j];
      }
    }
  }
  return 0;
}

std::vector<std::vector<float>> rotation_box_3d(
    std::vector<std::vector<float>> &corner,
    std::vector<std::vector<float>> &rot_mat_T,
    float &x_mean,
    float &y_mean) {
  std::vector<std::vector<float>> rot_corner(1, std::vector<float>(3, 0.0));
  matrix_mul(corner, rot_mat_T, rot_corner);
  rot_corner[0][0] += x_mean;
  rot_corner[0][1] += y_mean;
  return rot_corner;
}

namespace hobot {
namespace centerpoint {

Centerpoint_Publisher::Centerpoint_Publisher(rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr sp_pub)
                      : centerpoint_pub_(sp_pub) {

}

int Centerpoint_Publisher::publish(std::string &pointclude_file, std::shared_ptr<Perception> &perception) {
  cv::Mat ori_image;
  float width_offset = 0.0;
  float height_offset = 0.0;
  int width_resize = 0;
  int height_resize = 0;
  int width = 0;
  int height = 0;
  LoadLidarFile(pointclude_file, ori_image, width_offset, height_offset, width_resize, height_resize, width, height);
  Draw_perception(perception, ori_image, width_offset, height_offset, width_resize, height_resize, width, height);

  // 画布大小默认为1920 * 1080 ，需要将自车移动到画布最中间位置
  cv::Mat image(PUBIMAGE_HEIGHT, PUBIMAGE_WIDTH, ori_image.type());
  int centerpoint_x = PUBIMAGE_WIDTH/2;
  int centerpoint_y = PUBIMAGE_HEIGHT/2;
  int offset_x = centerpoint_x - selfCar_point_x;
  int offset_y = centerpoint_y - selfCar_point_y;
  cv::Scalar colorborder(255, 255, 255); // 白色填充
  cv::Mat warp_matrix = (cv::Mat_<float>(2, 3) <<
        cos(0), -sin(0), offset_x,
        sin(0), cos(0), offset_y);
  cv::warpAffine(ori_image, image, warp_matrix, image.size(), cv::INTER_LINEAR, cv::BORDER_CONSTANT, colorborder);

  auto msg = sensor_msgs::msg::Image();
  struct timespec time_start = {0, 0};
  clock_gettime(CLOCK_REALTIME, &time_start);
  msg.header.stamp.sec = time_start.tv_sec;
  msg.header.stamp.nanosec = time_start.tv_nsec;
  msg.encoding = "jpeg";
  msg.width = image.cols;
  msg.height = image.rows;

  // 使用opencv的imencode接口将mat转成vector，获取图片size
  std::vector<int> param;
  std::vector<uint8_t> jpeg;
  imencode(".jpg", image, jpeg, param);
  int32_t data_len = jpeg.size();
  msg.data.resize(data_len);
  memcpy(&msg.data[0], jpeg.data(), data_len);

  centerpoint_pub_->publish(msg);
  return 0;
}

// 加载雷达文件并渲染2d点云图
int Centerpoint_Publisher::LoadLidarFile(std::string &pointclude_file, cv::Mat &image_,
                      float &width_offset, float &height_offset,
                      int &width_resize, int &height_resize,
                      int &width, int &height) {
  char *data_buffer = nullptr;
  int32_t data_length = 0;
  auto ret = read_binary_file(pointclude_file, &data_buffer, &data_length);
  if(ret !=0 ) {
    rclcpp::shutdown();
    return -1;
  }
  int element_size = data_length / 4; // 点云文件中为float数据
  std::vector<float> padding_points(element_size);
  memcpy(padding_points.data(), data_buffer, data_length);
  delete [] data_buffer;
  
  // 2. remove padding
  int point_num = element_size / 5; // 每个点有五个维度的信息
  std::vector<float> points_y;
  std::vector<float> points_x;
  for (int i = 0; i < point_num; ++i) {
    // padding data: [-100, -100, -100, -100]
    if (padding_points[i * 5] == -100.f) {
      break;
    }
    points_y.push_back(padding_points[i * 5 + 1]);
    points_x.push_back(-1 * padding_points[i * 5 + 0]);
  }

  // 3. min_width, max_width
  std::vector<float>::iterator smallest_y =
      std::min_element(std::begin(points_y), std::end(points_y));
  std::vector<float>::iterator biggest_y =
      std::max_element(std::begin(points_y), std::end(points_y));

  // 4. min_height, max_height
  std::vector<float>::iterator smallest_x =
      std::min_element(std::begin(points_x), std::end(points_x));
  std::vector<float>::iterator biggest_x =
      std::max_element(std::begin(points_x), std::end(points_x));

  width_offset =
      *(smallest_y) < 0.0f ? std::ceil(std::abs(*(smallest_y))) + 10.f : 10.f;
  height_offset =
      *(smallest_x) < 0.0f ? std::ceil(std::abs(*(smallest_x))) + 10.f : 10.f;

  width = std::ceil((*(biggest_y) - *(smallest_y)) + 1.f) + 20;
  width_resize = 10;
  height = std::ceil((*(biggest_x) - *(smallest_x)) + 1.f) + 20;
  height_resize = 10;

  cv::Mat image(height * height_resize,
                width * width_resize,
                CV_8UC3,
                cv::Scalar(255, 255, 255));

  for (size_t i = 0; i < points_x.size(); ++i) {
    image.at<cv::Vec3b>(
        cv::Point2f((points_y[i] + width_offset) * width_resize,
                    (height - (points_x[i] + height_offset)) * height_resize))[0] = 255;
    image.at<cv::Vec3b>(
        cv::Point2f((points_y[i] + width_offset) * width_resize,
                    (height - (points_x[i] + height_offset)) * height_resize))[1] = 0;
    image.at<cv::Vec3b>(
        cv::Point2f((points_y[i] + width_offset) * width_resize,
                    (height - (points_x[i] + height_offset)) * height_resize))[2] = 0;
  }

  // 获取自车坐标
  selfCar_point_x = width_offset * width_resize;
  selfCar_point_y = (height - height_offset) * height_resize;

  image_ = image;
  return 0;
}

// 将检测结果渲染到2d点云图上
int Centerpoint_Publisher::Draw_perception(std::shared_ptr<Perception> &perception, cv::Mat &image,
                      float &width_offset, float &height_offset, int &width_resize, int &height_resize,int &width, int &height) {

  std::vector<LidarDetection3D> &dets = perception->lidar3d;
  std::vector<float> corners;
  std::vector<float> angles;
  std::vector<float> scores;

  for (size_t i = 0; i < dets.size(); ++i) {
    float x = dets[i].bbox.xs;
    float y = dets[i].bbox.ys;
    float z = dets[i].bbox.height;
    float w = dets[i].bbox.dim_0;
    float h = dets[i].bbox.dim_1;
    float p = dets[i].bbox.dim_2;
    float angle = -dets[i].bbox.rot;

    float rot_sin = std::sin(angle);
    float rot_cos = std::cos(angle);
    std::vector<std::vector<float>> rot_mat_T(3, std::vector<float>(3));
    rot_mat_T[0][0] = rot_cos;
    rot_mat_T[0][1] = -rot_sin;
    rot_mat_T[0][2] = 0;
    rot_mat_T[1][0] = rot_sin;
    rot_mat_T[1][1] = rot_cos;
    rot_mat_T[1][2] = 0;
    rot_mat_T[2][0] = 0;
    rot_mat_T[2][1] = 0;
    rot_mat_T[2][2] = 1;

    std::vector<std::vector<float>> corner(1, std::vector<float>(3));
    corner[0][0] = w / 2.0;
    corner[0][1] = -h / 2.0;
    corner[0][2] = z - p / 2.0;
    std::vector<std::vector<float>> rot_corner =
        rotation_box_3d(corner, rot_mat_T, x, y);
    corners.push_back(rot_corner[0][0]);
    corners.push_back(rot_corner[0][1]);
    corners.push_back(rot_corner[0][2]);

    corner[0][0] = w / 2.0;
    corner[0][1] = h / 2.0;
    corner[0][2] = z - p / 2.0;
    rot_corner = rotation_box_3d(corner, rot_mat_T, x, y);
    corners.push_back(rot_corner[0][0]);
    corners.push_back(rot_corner[0][1]);
    corners.push_back(rot_corner[0][2]);

    corner[0][0] = -w / 2.0;
    corner[0][1] = h / 2.0;
    corner[0][2] = z - p / 2.0;
    rot_corner = rotation_box_3d(corner, rot_mat_T, x, y);
    corners.push_back(rot_corner[0][0]);
    corners.push_back(rot_corner[0][1]);
    corners.push_back(rot_corner[0][2]);

    corner[0][0] = -w / 2.0;
    corner[0][1] = -h / 2.0;
    corner[0][2] = z - p / 2.0;
    rot_corner = rotation_box_3d(corner, rot_mat_T, x, y);
    corners.push_back(rot_corner[0][0]);
    corners.push_back(rot_corner[0][1]);
    corners.push_back(rot_corner[0][2]);

    corner[0][0] = w / 2.0;
    corner[0][1] = -h / 2.0;
    corner[0][2] = z + p / 2.0;
    rot_corner = rotation_box_3d(corner, rot_mat_T, x, y);
    corners.push_back(rot_corner[0][0]);
    corners.push_back(rot_corner[0][1]);
    corners.push_back(rot_corner[0][2]);

    corner[0][0] = w / 2.0;
    corner[0][1] = h / 2.0;
    corner[0][2] = z + p / 2.0;
    rot_corner = rotation_box_3d(corner, rot_mat_T, x, y);
    corners.push_back(rot_corner[0][0]);
    corners.push_back(rot_corner[0][1]);
    corners.push_back(rot_corner[0][2]);

    corner[0][0] = -w / 2.0;
    corner[0][1] = h / 2.0;
    corner[0][2] = z + p / 2.0;
    rot_corner = rotation_box_3d(corner, rot_mat_T, x, y);
    corners.push_back(rot_corner[0][0]);
    corners.push_back(rot_corner[0][1]);
    corners.push_back(rot_corner[0][2]);

    corner[0][0] = -w / 2.0;
    corner[0][1] = -h / 2.0;
    corner[0][2] = z + p / 2.0;
    rot_corner = rotation_box_3d(corner, rot_mat_T, x, y);
    corners.push_back(rot_corner[0][0]);
    corners.push_back(rot_corner[0][1]);
    corners.push_back(rot_corner[0][2]);

    angles.push_back(angle);
    scores.push_back(dets[i].score);
  }
  float score_thresh = 0.4;
  std::vector<float> box(16, 0);

  for (size_t idx = 0; idx < angles.size(); ++idx) {
    for (int p = 0; p < 8; ++p) {
      box[p * 2 + 0] = corners[idx * 8 * 3 + p * 3 + 0];
      box[p * 2 + 1] = corners[idx * 8 * 3 + p * 3 + 1];
    }
    if (scores[idx] < score_thresh) {
        continue;
    }

    // box < 0
    for (int k = 0; k < 4; ++k) {
      int i = k;
      int j = (k + 1) % 4;
      float pointi_y = box[i * 2 + 1];
      float pointj_y = box[j * 2 + 1];

      float pointi_x = -box[i * 2 + 0];
      float pointj_x = -box[j * 2 + 0];

      cv::line(
          image,
          cv::Point2f((pointi_y + width_offset) * width_resize,
                      (height - pointi_x - height_offset) * height_resize),
          cv::Point2f((pointj_y + width_offset) * width_resize,
                      (height - pointj_x - height_offset) * height_resize),
          cv::Scalar(0, 255, 0),
          1,
          cv::LINE_AA);
    }
    // direction
    float length = 4;
    float axis_rot = 0;
    std::vector<float> box_xy(box.begin(), box.begin() + 8);
    float x0 = -(box_xy[0] + box_xy[2] + box_xy[4] + box_xy[6]) / 4.0;
    float y0 = (box_xy[1] + box_xy[3] + box_xy[5] + box_xy[7]) / 4.0;
    float dx = -std::cos(angles[idx] + axis_rot) * length;
    float dy = -std::sin(angles[idx] + axis_rot) * length;
    float x1 = x0 + dx;
    float y1 = y0 + dy;

    cv::arrowedLine(
        image,
        cv::Point2f((y0 + width_offset) * width_resize,
                    (height - x0 - height_offset) * height_resize),
        cv::Point2f((y1 + width_offset) * width_resize,
                    (height - x1 - height_offset) * height_resize),
        cv::Scalar(0, 0, 255),
        1,
        cv::LINE_AA);
  }
  return 0;
}

} // namespace centerpoint
} // namespace hobot
