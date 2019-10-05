// Copyright (c) 2019 Yilin Gui. All rights reserved.
//
// filename: base_image_warping.h

#ifndef BASE_IMAGE_WARPING_H_
#define BASE_IMAGE_WARPING_H_

#include <vector>
#include <memory>
#include <Eigen/Dense>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

namespace image_warping {

class BaseImageWarping {
 public:
  BaseImageWarping();
  virtual ~BaseImageWarping();

  void SetAnchorPoints(const std::vector<Eigen::Vector2f> &src_pts,
      const std::vector<Eigen::Vector2f> &tgt_pts);

  void WarpImage(cv::Mat *image);

  void WarpImageWithTriangulation(cv::Mat *image);

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

  virtual void SolveTransformations() = 0;

  void GenerateRandomKeyPointsAndDoTriangulation(int width, int height);

  void GetBarycentricCoordinates(const Eigen::Vector2f &p1,
      const Eigen::Vector2f &p2, const Eigen::Vector2f &p3,
      const Eigen::Vector2f &p, Eigen::Vector3f *bc_coords);

 protected:
  std::vector<Eigen::Vector2f> source_points_;
  std::vector<Eigen::Vector2f> target_points_;
  // paint_mask_[i][j] = 1 means pixel (i, j) is painted, else in 'hole'
  std::vector<std::vector<int> > paint_mask_;
  cv::Mat image_mat_backup_;

  std::vector<Eigen::Vector2f> keypoints_;
  std::vector<Eigen::Vector2f> transformed_keypoints_;
  std::shared_ptr<cv::Subdiv2D> subdiv_2d_;
};

}  // namespace image_warping

#endif  // BASE_IMAGE_WARPING_H_
