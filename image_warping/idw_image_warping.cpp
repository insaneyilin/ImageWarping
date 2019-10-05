// Copyright (c) 2019 Yilin Gui. All rights reserved.
//
// filename: idw_image_warping.cpp

#include "image_warping/idw_image_warping.h"

namespace image_warping {

IDWImageWarping::IDWImageWarping() {
}

IDWImageWarping::~IDWImageWarping() {
}

void IDWImageWarping::WarpImage(cv::Mat *image) {
  const int width = image->cols;
  const int height = image->rows;
  paint_mask_.clear();
  paint_mask_.resize(height, std::vector<int>(width, 0));

}

Eigen::Vector2f IDWImageWarping::GetTransformedPoint(
    const Eigen::Vector2f &pt) {
  Eigen::Vector2f trans_pt;
  return trans_pt;
}

}  // namespace image_warping
