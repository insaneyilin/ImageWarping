// Copyright (c) 2019 Yilin Gui. All rights reserved.
//
// filename: idw_image_warping.cpp

#include "image_warping/idw_image_warping.h"

#include <cmath>
#include <iostream>

namespace image_warping {

IDWImageWarping::IDWImageWarping() {
}

IDWImageWarping::~IDWImageWarping() {
}

Eigen::Vector2f IDWImageWarping::GetTransformedPoint(
    const Eigen::Vector2f &pt) {
  const int num_ctrl_pts = source_points_.size();
  Eigen::Vector2f trans_pt;
  trans_pt[0] = 0.f;
  trans_pt[1] = 0.f;
  CalcWeights(pt);
  for (int i = 0; i < num_ctrl_pts; ++i) {
    trans_pt += weights_[i] *
        (target_points_[i] +
            local_trans_mat_list_[i] * (pt - source_points_[i]));
  }
  return trans_pt;
}

void IDWImageWarping::SolveTransformations() {
  SolveOptimalLocalTransformations();
}

void IDWImageWarping::CalcWeights(const Eigen::Vector2f &pt) {
  const int num_ctrl_pts = source_points_.size();
  float weights_sum = 0.f;
  weights_.clear();
  for (int i = 0; i < num_ctrl_pts; ++i) {
    float w = std::pow(Distance(pt, source_points_[i]), -2.f);
    weights_.push_back(w);
    weights_sum += w;
  }
  const float eps = 1e-5;
  for (int i = 0; i < num_ctrl_pts; ++i) {
    weights_[i] /= weights_sum;
  }
}

void IDWImageWarping::SolveOptimalLocalTransformations() {
  const int num_ctrl_pts = source_points_.size();
  local_trans_mat_list_.clear();
  local_trans_mat_list_.resize(num_ctrl_pts);
  for (int i = 0; i < num_ctrl_pts; ++i) {
    local_trans_mat_list_[i] = Eigen::Matrix2f::Identity();
  }
  if (num_ctrl_pts == 1) {
    return;
  } else {
    for (int i = 0; i < num_ctrl_pts; ++i) {
      Eigen::MatrixXf coeff_mat(2, 2);
      coeff_mat.setZero();
      Eigen::VectorXf b_vec_1(2);
      Eigen::VectorXf b_vec_2(2);
      b_vec_1.setZero();
      b_vec_2.setZero();
      for (int j = 0; j < num_ctrl_pts; ++j) {
        if (i == j) {
          continue;
        }
        float sigma = std::pow(
            Distance(source_points_[i], source_points_[j]), -2.f);
        coeff_mat(0, 0) += sigma * (source_points_[j][0] - source_points_[i][0]) *
            (source_points_[j][0] - source_points_[i][0]);
        coeff_mat(0, 1) += sigma * (source_points_[j][0] - source_points_[i][0]) *
            (source_points_[j][1] - source_points_[i][1]);
        coeff_mat(1, 0) += sigma * (source_points_[j][1] - source_points_[i][1]) *
            (source_points_[j][0] - source_points_[i][0]);
        coeff_mat(1, 1) += sigma * (source_points_[j][1] - source_points_[i][1]) *
            (source_points_[j][1] - source_points_[i][1]);

        b_vec_1(0) += sigma * (source_points_[j][0] - source_points_[i][0]) *
            (target_points_[j][0] - target_points_[i][0]);
        b_vec_1(1) += sigma * (source_points_[j][1] - source_points_[i][1]) *
            (target_points_[j][0] - target_points_[i][0]);
        b_vec_2(0) += sigma * (source_points_[j][0] - source_points_[i][0]) *
            (target_points_[j][1] - target_points_[i][1]);
        b_vec_2(1) += sigma * (source_points_[j][1] - source_points_[i][1]) *
            (target_points_[j][1] - target_points_[i][1]);
      }
      b_vec_1 = coeff_mat.colPivHouseholderQr().solve(b_vec_1);
      b_vec_2 = coeff_mat.colPivHouseholderQr().solve(b_vec_2);
      local_trans_mat_list_[i](0, 0) = b_vec_1(0);
      local_trans_mat_list_[i](0, 1) = b_vec_1(1);
      local_trans_mat_list_[i](1, 0) = b_vec_2(0);
      local_trans_mat_list_[i](1, 1) = b_vec_2(1);
    }
  }
}

}  // namespace image_warping
