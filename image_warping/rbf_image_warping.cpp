// Copyright (c) 2019 Yilin Gui. All rights reserved.
//
// filename: rbf_image_warping.cpp

#include "image_warping/rbf_image_warping.h"

#include <cmath>
#include <limits>
#include <iostream>

namespace image_warping {

RBFImageWarping::RBFImageWarping() {
}

RBFImageWarping::~RBFImageWarping() {
}

void RBFImageWarping::WarpImage(cv::Mat *image) {
  const int width = image->cols;
  const int height = image->rows;

  // get min radius list
  min_radius_list_.clear();
  for (int i = 0; i < source_points_.size(); ++i) {
    min_radius_list_.push_back(CalcMinRadius(i));
  }

  // solve linear system to get coeffs
  SolveLinearSystem();

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

Eigen::Vector2f RBFImageWarping::GetTransformedPoint(
    const Eigen::Vector2f &pt) {
  const int num_ctrl_pts = source_points_.size();
  Eigen::Vector2f trans_pt;
  trans_pt[0] = 0.f;
  trans_pt[1] = 0.f;
  for (int i = 0; i < num_ctrl_pts; ++i) {
    float dist = Distance(source_points_[i], pt);
    float g_i = HardyMultiQuadricFunction(dist, i);
    trans_pt[0] += g_i * alpha_x_list_[i];
    trans_pt[1] += g_i * alpha_y_list_[i];
  }
  trans_pt[0] += pt[0];
  trans_pt[1] += pt[1];
  return trans_pt;
}

float RBFImageWarping::HardyMultiQuadricFunction(float distance, int i) {
  return std::sqrt(distance * distance +
      min_radius_list_[i] * min_radius_list_[i]);
}

float RBFImageWarping::CalcMinRadius(int i) {
  float min_dist = std::numeric_limits<float>::max();
  const int num_ctrl_pts = source_points_.size();
  if (source_points_.size() <= 1) {
    return 0.f;
  }
  for (int j = 0; j < num_ctrl_pts; ++j) {
    if (i == j) {
      continue;
    }
    float dist = Distance(source_points_[j], target_points_[j]);
    if (dist < min_dist) {
      min_dist = dist;
    }
  }
  return min_dist;
}

void RBFImageWarping::SolveLinearSystem() {
  const int num_ctrl_pts = source_points_.size();
  if (num_ctrl_pts == 0) {
    return;
  }
  coeff_mat_.resize(num_ctrl_pts, num_ctrl_pts);
  alpha_x_vec_.resize(num_ctrl_pts);
  alpha_y_vec_.resize(num_ctrl_pts);
  diff_x_vec_.resize(num_ctrl_pts);
  diff_y_vec_.resize(num_ctrl_pts);
  for (int i = 0; i < num_ctrl_pts; ++i) {
    for (int j = 0; j < num_ctrl_pts; ++j) {
      float dist = Distance(source_points_[i], source_points_[j]);
      coeff_mat_(i, j) = HardyMultiQuadricFunction(dist, i);
    }
    diff_x_vec_(i) = target_points_[i][0] - source_points_[i][0];
    diff_y_vec_(i) = target_points_[i][1] - source_points_[i][1];
  }
  alpha_x_vec_ = coeff_mat_.colPivHouseholderQr().solve(diff_x_vec_);
  alpha_y_vec_ = coeff_mat_.colPivHouseholderQr().solve(diff_y_vec_);
  // special handling if there is only one pair of ctrl points
  if (num_ctrl_pts == 1) {
    const float coeff = coeff_mat_(0, 0) + 1e-5f;
    alpha_x_vec_(0) = diff_x_vec_(0) / coeff;
    alpha_y_vec_(0) = diff_y_vec_(0) / coeff;
  }
  alpha_x_list_.clear();
  alpha_y_list_.clear();
  for (int i = 0; i < num_ctrl_pts; ++i) {
    alpha_x_list_.push_back(alpha_x_vec_(i));
    alpha_y_list_.push_back(alpha_y_vec_(i));
  }
}

}  // namespace image_warping
