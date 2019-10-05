// Copyright (c) 2019 Yilin Gui. All rights reserved.
//
// filename: base_image_warping.h

#ifndef BASE_IMAGE_WARPING_H_
#define BASE_IMAGE_WARPING_H_

#include <vector>
#include <Eigen/Dense>
#include <opencv2/core/core.hpp>

namespace image_warping {

class BaseImageWarping {
 public:
  BaseImageWarping();
  virtual ~BaseImageWarping();

  void SetAnchorPoints(const std::vector<Eigen::Vector2f> &src_pts,
      const std::vector<Eigen::Vector2f> &tgt_pts);

  virtual void WarpImage(cv::Mat *image) = 0;

  virtual std::string Name() const {
    return "BaseImageWarping";
  }

 protected:
  virtual Eigen::Vector2f GetTransformedPoint(const Eigen::Vector2f &pt) = 0;
  void FillHole(cv::Mat *image);

  float Distance(const Eigen::Vector2f &p1, const Eigen::Vector2f &p2);

  bool IsValidImagePoint(const Eigen::Vector2f &pt, int width, int height) {
    int x = static_cast<int>(pt[0]);
    int y = static_cast<int>(pt[1]);
    return x >= 0 && x < width && y >= 0 && y < height;
  }

 protected:
  std::vector<Eigen::Vector2f> source_points_;
  std::vector<Eigen::Vector2f> target_points_;
  // paint_mask_[i][j] = 1 means pixel (i, j) is painted, else in 'hole'
  std::vector<std::vector<int> > paint_mask_;
  cv::Mat image_mat_backup_;
};

}  // namespace image_warping

#endif  // BASE_IMAGE_WARPING_H_
