// Copyright (c) 2019 Yilin Gui. All rights reserved.
//
// filename: idw_image_warping.h
//
// Image warping using Inverse Distance Weighted interpolation
//
// Input a list of control point pairs {(p_i, q_i)},
//

#ifndef IDW_IMAGE_WARPING_H_
#define IDW_IMAGE_WARPING_H_

#include "image_warping/base_image_warping.h"

namespace image_warping {

class IDWImageWarping : public BaseImageWarping {
 public:
  IDWImageWarping();
  virtual ~IDWImageWarping();

  virtual void WarpImage(cv::Mat *image) override;

  virtual std::string Name() const override {
    return "IDWImageWarping";
  }

 protected:
  virtual Eigen::Vector2f GetTransformedPoint(
      const Eigen::Vector2f &pt) override;
};

}  // namespace image_warping

#endif  // IDW_IMAGE_WARPING_H_
