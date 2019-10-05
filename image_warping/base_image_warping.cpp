// Copyright (c) 2019 Yilin Gui. All rights reserved.
//
// filename: base_image_warping.cpp

#include "image_warping/base_image_warping.h"

#include <cmath>
#include <limits>
#include <iostream>

namespace image_warping {

BaseImageWarping::BaseImageWarping() {
  source_points_.reserve(100);
  target_points_.reserve(100);
}

BaseImageWarping::~BaseImageWarping() {
}

void BaseImageWarping::SetAnchorPoints(
    const std::vector<Eigen::Vector2f> &src_pts,
    const std::vector<Eigen::Vector2f> &tgt_pts) {
  source_points_.clear();
  source_points_.assign(src_pts.begin(), src_pts.end());
  target_points_.clear();
  target_points_.assign(tgt_pts.begin(), tgt_pts.end());

  SolveTransformations();
}

void BaseImageWarping::WarpImage(cv::Mat *image) {
  const int width = image->cols;
  const int height = image->rows;

  paint_mask_.clear();
  paint_mask_.resize(height, std::vector<int>(width, 0));
  image_mat_backup_ = image->clone();
  image->setTo(cv::Scalar(255, 255, 255));

  Eigen::Vector2f pt;
  Eigen::Vector2f trans_pt;
  for (int i = 0; i < height; ++i) {
    for (int j = 0; j < width; ++j) {
      pt[0] = j;
      pt[1] = i;
      trans_pt = GetTransformedPoint(pt);
      if (!IsValidImagePoint(trans_pt, width, height)) {
        continue;
      }
      int trans_x = static_cast<int>(trans_pt[0]);
      int trans_y = static_cast<int>(trans_pt[1]);
      image->at<cv::Vec3b>(trans_y, trans_x) =
          image_mat_backup_.at<cv::Vec3b>(i, j);
      paint_mask_[trans_y][trans_x] = 1;
    }
  }
  FillHole(image);
}

void BaseImageWarping::FillHole(cv::Mat *image) {
  const int width = image->cols;
  const int height = image->rows;
  const int half_ksize = 3;
  for (int i = 0; i < height; ++i) {
    for (int j = 0; j < width; ++j) {
      if (paint_mask_[i][j] == 1) {
        continue;
      }
      int min_dist = std::numeric_limits<int>::max();
      int min_dist_i = -1;
      int min_dist_j = -1;
      const int start_ii = std::max(0, i - half_ksize);
      const int end_ii = std::min(i + half_ksize, height);
      const int start_jj = std::max(0, j - half_ksize);
      const int end_jj = std::min(j + half_ksize, width);
      for (int ii = start_ii; ii < end_ii; ++ii) {
        for (int jj = start_jj; jj < end_jj; ++jj) {
          if (paint_mask_[ii][jj] == 0) {
            continue;
          }
          int dist = std::hypot(std::abs(ii - i), std::abs(jj - j));
          if (dist < min_dist) {
            min_dist = dist;
            min_dist_i = ii;
            min_dist_j = jj;
          }
        }
      }
      if (min_dist_i < 0 || min_dist_j < 0) {
        continue;
      }
      image->at<cv::Vec3b>(i, j) =
          image->at<cv::Vec3b>(min_dist_i, min_dist_j);
    }
  }
}

float BaseImageWarping::Distance(
    const Eigen::Vector2f &p1, const Eigen::Vector2f &p2) {
  float diff_x = p1[0] - p2[0];
  float diff_y = p1[1] - p2[1];
  return std::hypot(std::fabs(diff_x), std::fabs(diff_y));
}

}  // namespace image_warping
