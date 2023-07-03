// Copyright (c) 2020 Horizon Robotics.All Rights Reserved.
//
// The material in this file is confidential and contains trade secrets
// of Horizon Robotics Inc. This is proprietary information owned by
// Horizon Robotics Inc. No part of this work may be disclosed,
// reproduced, copied, transmitted, or used in any way for any purpose,
// without the express written permission of Horizon Robotics Inc.

#include "utils/box3d_nms.h"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <iostream>
#include <unordered_map>
#include <vector>

bool Compare(const std::pair<float, int> &a, const std::pair<float, int> &b) {
  if (a.first != b.first) {
    return a.first > b.first;
  } else {
    return a.second < b.second;
  }
}

void Lidar3DNMS(std::vector<int> &keep_res,
                std::vector<std::vector<float>> &boxes,
                std::vector<float> &scores,
                float thresh,
                int per_max_size,
                int post_max_size) {
  assert(boxes[0].size() == 5);
  std::vector<int> order;
  std::vector<std::vector<float>> boxes_order_res;
  std::vector<float> scores_order_res;

  std::vector<std::pair<float, int>> tmp_scores;
  for (size_t i = 0; i < scores.size(); ++i) {
    tmp_scores.emplace_back(scores[i], i);
  }
  std::sort(tmp_scores.begin(), tmp_scores.end(), Compare);

  if (tmp_scores.size() > per_max_size) {
    for (int i = 0; i < per_max_size; i++) {
      int tmp_num = tmp_scores[i].second;
      order.push_back(tmp_num);
      boxes_order_res.push_back(boxes[tmp_num]);
      scores_order_res.push_back(scores[tmp_num]);
    }
  } else {
    for (size_t i = 0; i < tmp_scores.size(); i++) {
      int tmp_num = tmp_scores[i].second;
      order.push_back(tmp_num);
      boxes_order_res.push_back(boxes[tmp_num]);
      scores_order_res.push_back(scores[tmp_num]);
    }
  }

  // xyxyr -> back to xywhr
  // note: better skip this step before nms_bev call in the future

  std::vector<std::vector<float>> boxes_res(boxes.size(),
                                            std::vector<float>(5, 0));
  // torch.stack
  for (size_t i = 0; i < boxes_order_res.size(); i++) {
    boxes_res[i][0] = (boxes_order_res[i][0] + boxes_order_res[i][2]) / 2;
    boxes_res[i][1] = (boxes_order_res[i][1] + boxes_order_res[i][3]) / 2;
    boxes_res[i][2] = (boxes_order_res[i][2] - boxes_order_res[i][0]);
    boxes_res[i][3] = (boxes_order_res[i][3] - boxes_order_res[i][1]);
    boxes_res[i][4] = boxes_order_res[i][4];
  }

  std::vector<int> keep(boxes_res.size());
  Lidar3DNmsRotated(keep, boxes_res, scores_order_res, thresh);

  if (keep.size() > post_max_size) {
    for (int i = 0; i < post_max_size; i++) {
      keep_res.push_back(order[keep[i]]);
    }
  } else {
    for (size_t i = 0; i < keep.size(); i++) {
      keep_res.push_back(order[keep[i]]);
    }
  }
}

void Lidar3DNmsRotated(std::vector<int> &keep,
                       std::vector<std::vector<float>> &dets,
                       std::vector<float> &scores,
                       float iou_threshold) {
  std::vector<std::pair<float, int>> tmp_scores;
  for (size_t i = 0; i < scores.size(); ++i) {
    tmp_scores.emplace_back(scores[i], i);
  }
  std::sort(tmp_scores.begin(), tmp_scores.end(), Compare);

  std::vector<int> order_t;
  for (size_t i = 0; i < tmp_scores.size(); i++) {
    order_t.push_back(tmp_scores[i].second);
  }

  int ndets = dets.size();
  std::vector<std::vector<int>> suppressed_t(ndets, std::vector<int>(1, 0));
  std::vector<std::vector<int>> keep_t(ndets, std::vector<int>(1, 0));

  std::vector<int> suppressed(ndets);
  std::vector<int> order(scores.size());

  for (int i = 0; i < ndets; i++) {
    suppressed[i] = suppressed_t[i][0];
    keep[i] = keep_t[i][0];
  }
  for (size_t i = 0; i < order_t.size(); i++) {
    order[i] = order_t[i];
  }

  int64_t num_to_keep = 0;
  for (int _i = 0; _i < ndets; _i++) {
    auto i = order[_i];
    if (suppressed[i] == 1) {
      continue;
    }
    keep[num_to_keep++] = i;
    for (int _j = _i + 1; _j < ndets; _j++) {
      auto j = order[_j];
      if (suppressed[j] == 1) {
        continue;
      }

      float ovr = single_box_iou_rotated(dets[i], dets[j], 0);

      if (ovr >= iou_threshold) {
        suppressed[j] = 1;
      }
    }
  }

  keep.resize(num_to_keep);
}

template <typename T>
struct Point_box3d_nms {
  T x, y;
  explicit Point_box3d_nms(const T &px = 0, const T &py = 0) : x(px), y(py) {}
  Point_box3d_nms operator+(const Point_box3d_nms &p) const {
    return Point_box3d_nms(x + p.x, y + p.y);
  }
  Point_box3d_nms &operator+=(const Point_box3d_nms &p) {
    x += p.x;
    y += p.y;
    return *this;
  }
  Point_box3d_nms operator-(const Point_box3d_nms &p) const {
    return Point_box3d_nms(x - p.x, y - p.y);
  }
  Point_box3d_nms operator*(const T coeff) const {
    return Point_box3d_nms(x * coeff, y * coeff);
  }
};

template <typename T>
void get_rotated_vertices(RotatedBox &box, Point_box3d_nms<float> (&pts)[4]) {
  double theta = box.a;
  T cosTheta2 = (T)cos(theta) * 0.5f;
  T sinTheta2 = (T)sin(theta) * 0.5f;

  // y: top --> down; x: left --> right
  pts[0].x = box.x_ctr - sinTheta2 * box.h - cosTheta2 * box.w;
  pts[0].y = box.y_ctr + cosTheta2 * box.h - sinTheta2 * box.w;
  pts[1].x = box.x_ctr + sinTheta2 * box.h - cosTheta2 * box.w;
  pts[1].y = box.y_ctr - cosTheta2 * box.h - sinTheta2 * box.w;
  pts[2].x = 2 * box.x_ctr - pts[0].x;
  pts[2].y = 2 * box.y_ctr - pts[0].y;
  pts[3].x = 2 * box.x_ctr - pts[1].x;
  pts[3].y = 2 * box.y_ctr - pts[1].y;
}

template <typename T>
T dot_2d(const Point_box3d_nms<T> &A, const Point_box3d_nms<T> &B) {
  return A.x * B.x + A.y * B.y;
}

template <typename T>
T cross_2d(const Point_box3d_nms<T> &A, const Point_box3d_nms<T> &B) {
  return A.x * B.y - B.x * A.y;
}

template <typename T>
int get_intersection_points(const Point_box3d_nms<T> (&pts1)[4],
                            const Point_box3d_nms<T> (&pts2)[4],
                            Point_box3d_nms<T> (&intersections)[24]) {
  // Line vector
  // A line from p1 to p2 is: p1 + (p2-p1)*t, t=[0,1]
  Point_box3d_nms<T> vec1[4], vec2[4];
  for (int i = 0; i < 4; i++) {
    vec1[i] = pts1[(i + 1) % 4] - pts1[i];
    vec2[i] = pts2[(i + 1) % 4] - pts2[i];
  }

  // Line test - test all line combos for intersection
  int num = 0;  // number of intersections
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 4; j++) {
      // Solve for 2x2 Ax=b
      T det = cross_2d<T>(vec2[j], vec1[i]);

      // This takes care of parallel lines
      if (fabs(det) <= 1e-14) {
        continue;
      }

      auto vec12 = pts2[j] - pts1[i];

      T t1 = cross_2d<T>(vec2[j], vec12) / det;
      T t2 = cross_2d<T>(vec1[i], vec12) / det;

      if (t1 >= 0.0f && t1 <= 1.0f && t2 >= 0.0f && t2 <= 1.0f) {
        intersections[num++] = pts1[i] + vec1[i] * t1;
      }
    }
  }

  // Check for vertices of rect1 inside rect2
  {
    const auto &AB = vec2[0];
    const auto &DA = vec2[3];
    auto ABdotAB = dot_2d<T>(AB, AB);
    auto ADdotAD = dot_2d<T>(DA, DA);
    for (int i = 0; i < 4; i++) {
      // assume ABCD is the rectangle, and P is the point to be judged
      // P is inside ABCD iff. P's projection on AB lies within AB
      // and P's projection on AD lies within AD

      auto AP = pts1[i] - pts2[0];

      auto APdotAB = dot_2d<T>(AP, AB);
      auto APdotAD = -dot_2d<T>(AP, DA);

      if ((APdotAB >= 0) && (APdotAD >= 0) && (APdotAB <= ABdotAB) &&
          (APdotAD <= ADdotAD)) {
        intersections[num++] = pts1[i];
      }
    }
  }

  // Reverse the check - check for vertices of rect2 inside rect1
  {
    const auto &AB = vec1[0];
    const auto &DA = vec1[3];
    auto ABdotAB = dot_2d<T>(AB, AB);
    auto ADdotAD = dot_2d<T>(DA, DA);
    for (int i = 0; i < 4; i++) {
      auto AP = pts2[i] - pts1[0];

      auto APdotAB = dot_2d<T>(AP, AB);
      auto APdotAD = -dot_2d<T>(AP, DA);

      if ((APdotAB >= 0) && (APdotAD >= 0) && (APdotAB <= ABdotAB) &&
          (APdotAD <= ADdotAD)) {
        intersections[num++] = pts2[i];
      }
    }
  }

  return num;
}

template <typename T>
int convex_hull_graham(const Point_box3d_nms<T> (&p)[24],
                       const int &num_in,
                       Point_box3d_nms<T> (&q)[24],
                       bool shift_to_zero = false) {
  assert(num_in >= 2);

  // Step 1:
  // Find point with minimum y
  // if more than 1 points have the same minimum y,
  // pick the one with the minimum x.
  int t = 0;
  for (int i = 1; i < num_in; i++) {
    if (p[i].y < p[t].y || (p[i].y == p[t].y && p[i].x < p[t].x)) {
      t = i;
    }
  }
  auto &start = p[t];  // starting point

  // Step 2:
  // Subtract starting point from every points (for sorting in the next step)
  for (int i = 0; i < num_in; i++) {
    q[i] = p[i] - start;
  }

  // Swap the starting point to position 0
  auto tmp = q[0];
  q[0] = q[t];
  q[t] = tmp;

  // Step 3:
  // Sort point 1 ~ num_in according to their relative cross-product values
  // (essentially sorting according to angles)
  // If the angles are the same, sort according to their distance to origin
  T dist[24];
  for (int i = 0; i < num_in; i++) {
    dist[i] = dot_2d<T>(q[i], q[i]);
  }

  // CPU version
  std::sort(
      q + 1,
      q + num_in,
      [](const Point_box3d_nms<T> &A, const Point_box3d_nms<T> &B) -> bool {
        T temp = cross_2d<T>(A, B);
        if (fabs(temp) < 1e-6) {
          return dot_2d<T>(A, A) < dot_2d<T>(B, B);
        } else {
          return temp > 0;
        }
      });
  // compute distance to origin after sort, since the points are now different.
  for (int i = 0; i < num_in; i++) {
    dist[i] = dot_2d<T>(q[i], q[i]);
  }

  // Step 4:
  // Make sure there are at least 2 points (that don't overlap with each other)
  // in the stack
  int k;  // index of the non-overlapped second point
  for (k = 1; k < num_in; k++) {
    if (dist[k] > 1e-8) {
      break;
    }
  }
  if (k == num_in) {
    // We reach the end, which means the convex hull is just one point
    q[0] = p[t];
    return 1;
  }
  q[1] = q[k];
  int m = 2;  // 2 points in the stack
  // Step 5:
  // Finally we can start the scanning process.
  // When a non-convex relationship between the 3 points is found
  // (either concave shape or duplicated points),
  // we pop the previous point from the stack
  // until the 3-point relationship is convex again, or
  // until the stack only contains two points
  for (int i = k + 1; i < num_in; i++) {
    while (m > 1 && cross_2d<T>(q[i] - q[m - 2], q[m - 1] - q[m - 2]) >= 0) {
      m--;
    }
    q[m++] = q[i];
  }

  // Step 6 (Optional):
  // In general sense we need the original coordinates, so we
  // need to shift the points back (reverting Step 2)
  // But if we're only interested in getting the area/perimeter of the shape
  // We can simply return.
  if (!shift_to_zero) {
    for (int i = 0; i < m; i++) {
      q[i] += start;
    }
  }

  return m;
}

template <typename T>
T polygon_area(const Point_box3d_nms<T> (&q)[24], const int &m) {
  if (m <= 2) {
    return 0;
  }

  T area = 0;
  for (int i = 1; i < m - 1; i++) {
    area += fabs(cross_2d<T>(q[i] - q[0], q[i + 1] - q[0]));
  }

  return area / 2.0;
}

float rotated_boxes_intersection(RotatedBox &box1, RotatedBox &box2) {
  // There are up to 4 x 4 + 4 + 4 = 24 intersections (including dups) returned
  // from rotated_rect_intersection_pts
  Point_box3d_nms<float> intersectPts[24], orderedPts[24];

  Point_box3d_nms<float> pts1[4];
  Point_box3d_nms<float> pts2[4];
  get_rotated_vertices<float>(box1, pts1);
  get_rotated_vertices<float>(box2, pts2);

  int num = get_intersection_points<float>(pts1, pts2, intersectPts);
  if (num <= 2) {
    return 0.0;
  }

  // Convex Hull to order the intersection points in clockwise order and find
  // the contour area.
  int num_convex =
      convex_hull_graham<float>(intersectPts, num, orderedPts, true);
  return polygon_area<float>(orderedPts, num_convex);
}

float single_box_iou_rotated(std::vector<float> &box1_raw,
                             std::vector<float> &box2_raw,
                             int mode_flag) {
  // shift center to the middle point to achieve higher precision in result

  auto center_shift_x = (box1_raw[0] + box2_raw[0]) / 2.0;
  auto center_shift_y = (box1_raw[1] + box2_raw[1]) / 2.0;
  RotatedBox box1, box2;
  box1.x_ctr = box1_raw[0] - center_shift_x;
  box1.y_ctr = box1_raw[1] - center_shift_y;
  box1.w = box1_raw[2];
  box1.h = box1_raw[3];
  box1.a = box1_raw[4];
  box2.x_ctr = box2_raw[0] - center_shift_x;
  box2.y_ctr = box2_raw[1] - center_shift_y;
  box2.w = box2_raw[2];
  box2.h = box2_raw[3];
  box2.a = box2_raw[4];

  float area1 = box1.w * box1.h;
  float area2 = box2.w * box2.h;
  if (area1 < 1e-14 || area2 < 1e-14) {
    return 0.f;
  }

  float intersection = rotated_boxes_intersection(box1, box2);
  float baseS = 1.0;
  if (mode_flag == 0) {
    baseS = (area1 + area2 - intersection);
  } else if (mode_flag == 1) {
    baseS = area1;
  }
  // iou is float
  float iou = intersection / baseS;
  return iou;
}
