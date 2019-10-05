// Copyright (c) 2019 Yilin Gui. All rights reserved.
//
// filename: rbf_image_warping.h
//
// Image warping using Radial Basis Functions
//
// Input a list of control point pairs {(p_i, q_i)},
//

#ifndef RBF_IMAGE_WARPING_H_
#define RBF_IMAGE_WARPING_H_

#include "image_warping/base_image_warping.h"

namespace image_warping {

class RBFImageWarping : public BaseImageWarping {
 public:
  RBFImageWarping();
  virtual ~RBFImageWarping();

  virtual std::string Name() const override {
    return "RBFImageWarping";
  }

 protected:
  virtual Eigen::Vector2f GetTransformedPoint(
      const Eigen::Vector2f &pt) override;
  virtual void SolveTransformations() override;

 private:
  float HardyMultiQuadricFunction(float distance, int i);
  float CalcMinRadius(int i);
  void SolveLinearSystem();

 private:
  Eigen::MatrixXf coeff_mat_;
  Eigen::VectorXf alpha_x_vec_;
  Eigen::VectorXf alpha_y_vec_;
  Eigen::VectorXf diff_x_vec_;
  Eigen::VectorXf diff_y_vec_;

  std::vector<float> min_radius_list_;
  std::vector<float> alpha_x_list_;
  std::vector<float> alpha_y_list_;
};

}  // namespace image_warping

#endif  // RBF_IMAGE_WARPING_H_
