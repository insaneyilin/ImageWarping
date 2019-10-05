// Copyright (c) 2019 Yilin Gui. All rights reserved.
//
// filename: base_image_warping.cpp

#include "image_warping/base_image_warping.h"

#include <cmath>
#include <limits>
#include <iostream>

#include "image_warping/scanline.h"

namespace image_warping {

BaseImageWarping::BaseImageWarping() {
  source_points_.reserve(100);
  target_points_.reserve(100);
  keypoints_.reserve(10000);
  transformed_keypoints_.reserve(10000);
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

void BaseImageWarping::WarpImageWithTriangulation(cv::Mat *image) {
  const int width = image->cols;
  const int height = image->rows;

  paint_mask_.clear();
  paint_mask_.resize(height, std::vector<int>(width, 0));
  image_mat_backup_ = image->clone();
  image->setTo(cv::Scalar(255, 255, 255));

  GenerateRandomKeyPointsAndDoTriangulation(width, height);

  // get transformed keypoints
  Eigen::Vector2f trans_pt;
  transformed_keypoints_.clear();
  for (const auto &pt : keypoints_) {
    trans_pt = GetTransformedPoint(pt);
    transformed_keypoints_.push_back(trans_pt);
    if (!IsValidImagePoint(trans_pt, width, height)) {
      continue;
    }
    int orig_x = static_cast<int>(pt[0]);
    int orig_y = static_cast<int>(pt[1]);
    int trans_x = static_cast<int>(trans_pt[0]);
    int trans_y = static_cast<int>(trans_pt[1]);
    image->at<cv::Vec3b>(trans_y, trans_x) =
        image_mat_backup_.at<cv::Vec3b>(orig_y, orig_x);
    paint_mask_[trans_y][trans_x] = 1;
  }
  // warp triangles
  std::vector<cv::Vec6f> triangle_list;
  subdiv_2d_->getTriangleList(triangle_list);
  std::vector<Eigen::Vector2f> triangle(3);
  std::vector<Eigen::Vector2f> transformed_triangle(3);
  for(size_t i = 0; i < triangle_list.size(); ++i) {
    cv::Vec6f t = triangle_list[i];
    bool valid_triangle = true;
    for (int ii = 0; ii < 3; ++ii) {
      triangle[ii][0] = t[ii * 2];
      triangle[ii][1] = t[ii * 2 + 1];
      transformed_triangle[ii] = GetTransformedPoint(triangle[ii]);
      if (!IsValidImagePoint(transformed_triangle[ii], width, height)) {
        valid_triangle = false;
        continue;
      }
    }
    if (!valid_triangle) {
      continue;
    }
    std::vector<Eigen::Vector2i> internal_pts;
    ScanLineAlgorithm::GetInternalPointsOfPolygon(transformed_triangle, &internal_pts);
    for (size_t k = 0; k < internal_pts.size(); ++k) {
      Eigen::Vector2f in_pt = internal_pts[k].cast<float>();
      Eigen::Vector3f bc_coords;
      GetBarycentricCoordinates(transformed_triangle[0],
          transformed_triangle[1], transformed_triangle[2], in_pt,
          &bc_coords);
      Eigen::Vector2f orig_interal_pt =
          bc_coords[0] * triangle[0] + bc_coords[1] * triangle[1] +
              bc_coords[2] * triangle[2];
      if (!IsValidImagePoint(orig_interal_pt, width, height)) {
        continue;
      }
      int orig_x = static_cast<int>(orig_interal_pt[0]);
      int orig_y = static_cast<int>(orig_interal_pt[1]);
      int trans_x = static_cast<int>(in_pt[0]);
      int trans_y = static_cast<int>(in_pt[1]);
      image->at<cv::Vec3b>(trans_y, trans_x) =
          image_mat_backup_.at<cv::Vec3b>(orig_y, orig_x);
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

void BaseImageWarping::GenerateRandomKeyPointsAndDoTriangulation(
    int width, int height) {
  subdiv_2d_.reset(new cv::Subdiv2D(cv::Rect(0, 0, width, height)));
  keypoints_.clear();
  std::vector<cv::Point2f> cv_pt_vec;
  int num_inner_points = static_cast<int>(width * height * 0.001);
  Eigen::Vector2f pt;
  pt[0] = 0;
  pt[1] = 0;
  keypoints_.push_back(pt);
  cv_pt_vec.push_back(cv::Point2f(pt[0], pt[1]));
  pt[0] = width - 1;
  pt[1] = 0;
  keypoints_.push_back(pt);
  cv_pt_vec.push_back(cv::Point2f(pt[0], pt[1]));
  pt[0] = width - 1;
  pt[1] = height - 1;
  keypoints_.push_back(pt);
  cv_pt_vec.push_back(cv::Point2f(pt[0], pt[1]));
  pt[0] = 0;
  pt[1] = height - 1;
  keypoints_.push_back(pt);
  cv_pt_vec.push_back(cv::Point2f(pt[0], pt[1]));

  int num_pts_on_boundary = static_cast<int>(std::sqrt(num_inner_points));
  for (int i = 0; i < num_pts_on_boundary; ++i) {
    pt[0] = static_cast<float>(rand() % width);
    pt[1] = 0;
    keypoints_.push_back(pt);
    cv_pt_vec.push_back(cv::Point2f(pt[0], pt[1]));

    pt[0] = static_cast<float>(rand() % width);
    pt[1] = height - 1;
    keypoints_.push_back(pt);
    cv_pt_vec.push_back(cv::Point2f(pt[0], pt[1]));

    pt[0] = 0;
    pt[1] = static_cast<float>(rand() % height);
    keypoints_.push_back(pt);
    cv_pt_vec.push_back(cv::Point2f(pt[0], pt[1]));

    pt[0] = width - 1;
    pt[1] = static_cast<float>(rand() % height);
    keypoints_.push_back(pt);
    cv_pt_vec.push_back(cv::Point2f(pt[0], pt[1]));
  }
  for (int i = 0; i < num_inner_points; ++i) {
    pt[0] = static_cast<float>(rand() % width);
    pt[1] = static_cast<float>(rand() % height);
    keypoints_.push_back(pt);
    cv_pt_vec.push_back(cv::Point2f(pt[0], pt[1]));
  }
  subdiv_2d_->insert(cv_pt_vec);
}

void BaseImageWarping::GetBarycentricCoordinates(
    const Eigen::Vector2f &p1,
    const Eigen::Vector2f &p2, const Eigen::Vector2f &p3,
    const Eigen::Vector2f &p, Eigen::Vector3f *bc_coords) {
  const float x1 = p1[0];
  const float y1 = p1[1];
  const float x2 = p2[0];
  const float y2 = p2[1];
  const float x3 = p3[0];
  const float y3 = p3[1];
  const float x = p[0];
  const float y = p[1];

  const float x_minus_x3 = x - x3;
  const float y_minus_y3 = y - y3;
  const float x1_minus_x3 = x1 - x3;
  const float x3_minus_x2 = x3 - x2;
  const float y1_minus_y3 = y1 - y3;
  const float y2_minus_y3 = y2 - y3;
  const float y3_minus_y1 = y3 - y1;

  (*bc_coords)[0] = (y2_minus_y3 * x_minus_x3 + x3_minus_x2 * y_minus_y3) /
      (y2_minus_y3 * x1_minus_x3 + x3_minus_x2 * y1_minus_y3);
  (*bc_coords)[1] = (y3_minus_y1 * x_minus_x3 + x1_minus_x3 * y_minus_y3) /
      (y2_minus_y3 * x1_minus_x3 + x3_minus_x2 * y1_minus_y3);
  (*bc_coords)[2] = 1.f - (*bc_coords)[0] - (*bc_coords)[1];
}

}  // namespace image_warping
