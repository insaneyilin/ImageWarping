// Copyright (c) 2019 Yilin Gui. All rights reserved.
//
// filename: image_widget.h

#ifndef IMAGE_WIDGET_H_
#define IMAGE_WIDGET_H_

#include <QWidget>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>

#include <vector>
#include <string>
#include <unordered_map>
#include <memory>

#include "image_warping/base_image_warping.h"

QT_BEGIN_NAMESPACE
class QImage;
class QPainter;
QT_END_NAMESPACE

class ImageWidget : public QWidget {
  Q_OBJECT

 public:
  ImageWidget();
  ~ImageWidget();

 protected:
  void paintEvent(QPaintEvent *paintevent);

  void mousePressEvent(QMouseEvent *mouseevent);
  void mouseMoveEvent(QMouseEvent *mouseevent);
  void mouseReleaseEvent(QMouseEvent *mouseevent);

 public slots:
  /// File IO
  // Open an image file, support ".bmp, .png, .jpg" format
  void Open();
  // Save image to current file
  void Save();
  // Save image to another file
  void SaveAs();

  /// Image processing
  // Invert pixel value in image
  void Invert();
  // Mirror image vertically or horizontally
  void Mirror(bool horizontal = false, bool vertical = true);
  // convert to grayscale
  void ToGray();
  // Restore image to origin
  void Restore();

  /// Image Warping
  void Warp();

  void ClearControlPoints();

  void UndoSelect();

  void SetSelectMode(bool status);

 private:
  void InitWarpingInstanceMap();

 private:
  cv::Mat image_mat_;  // save image in rgb format
  cv::Mat image_mat_backup_;
  cv::Mat image_mat_bgr_;  // save image in bgr format, for writing to file

  bool is_drawing_ = false;
  bool select_mode_ = false;

  QPoint point_start_;
  QPoint point_end_;

  std::vector<Eigen::Vector2f> source_points_;
  std::vector<Eigen::Vector2f> target_points_;

  // warping method name ("IDW"/"RBF") -> warping instance
  std::unordered_map<std::string,
      std::shared_ptr<image_warping::BaseImageWarping>> warping_inst_map_;
  std::string warping_method_ = "RBF";
  image_warping::BaseImageWarping *image_warping_ = nullptr;
};

#endif  // IMAGE_WIDGET_H_
