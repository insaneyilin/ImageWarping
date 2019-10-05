// Copyright (c) 2019 Yilin Gui. All rights reserved.
//
// filename: image_widget.cpp

#include "image_widget.h"

#include <QImage>
#include <QPainter>
#include <QtWidgets>

#include <iostream>

#include <opencv2/highgui/highgui.hpp>

#include "image_warping/rbf_image_warping.h"
#include "image_warping/idw_image_warping.h"

ImageWidget::ImageWidget() {
  InitWarpingInstanceMap();
}

ImageWidget::~ImageWidget() {
}

void ImageWidget::paintEvent(QPaintEvent *paintevent) {
  QPainter painter;
  painter.begin(this);

  // Draw background
  painter.setBrush(Qt::lightGray);
  QRect back_rect(0, 0, width(), height());
  painter.drawRect(back_rect);

  // Draw image
  QImage image_show = QImage((unsigned char *)(image_mat_.data),
      image_mat_.cols, image_mat_.rows, image_mat_.step,
      QImage::Format_RGB888);
  QRect rect = QRect(0, 0, image_show.width(), image_show.height());
  painter.drawImage(rect, image_show);

  // Draw control lines
  if (select_mode_) {
    QPen pen(Qt::red, 4);
    painter.setPen(pen);
    const int num_ctrl_pts = source_points_.size();
    for (int i = 0; i < num_ctrl_pts; ++i) {
      QPoint p1(static_cast<int>(source_points_[i][0]),
          static_cast<int>(source_points_[i][1]));
      QPoint p2(static_cast<int>(target_points_[i][0]),
          static_cast<int>(target_points_[i][1]));
      painter.drawLine(p1, p2);
    }
    if (is_drawing_) {
      painter.drawLine(point_start_, point_end_);
    }
  }

  painter.end();
}

void ImageWidget::mousePressEvent(QMouseEvent *mouseevent) {
  if (select_mode_ && mouseevent->button() == Qt::LeftButton) {
    is_drawing_ = true;
    auto pt = mouseevent->pos();
    point_start_ = pt;
    point_end_ = pt;
    update();
  }
}

void ImageWidget::mouseMoveEvent(QMouseEvent *mouseevent) {
  if (select_mode_ && is_drawing_) {
    point_end_ = mouseevent->pos();

    if (realtime_warping_mode_) {
      std::vector<Eigen::Vector2f> src_pts = source_points_;
      std::vector<Eigen::Vector2f> tgt_pts = target_points_;
      Eigen::Vector2f pt;
      pt[0] = point_start_.x();
      pt[1] = point_start_.y();
      src_pts.push_back(pt);
      pt[0] = point_end_.x();
      pt[1] = point_end_.y();
      tgt_pts.push_back(pt);
      image_warping_ = warping_inst_map_[warping_method_].get();
      image_warping_->SetAnchorPoints(src_pts, tgt_pts);
      image_warping_->WarpImage(&image_mat_);
    }

    update();
  }
}

void ImageWidget::mouseReleaseEvent(QMouseEvent *mouseevent) {
  if (select_mode_ && is_drawing_) {
    Eigen::Vector2f pt;
    pt[0] = point_start_.x();
    pt[1] = point_start_.y();
    source_points_.push_back(pt);

    pt[0] = point_end_.x();
    pt[1] = point_end_.y();
    target_points_.push_back(pt);

    is_drawing_ = false;
    update();
  }
}

void ImageWidget::Open() {
  // Open file
  QString file_name = QFileDialog::getOpenFileName(this,
      tr("Read Image"), ".", tr("Images(*.bmp *.png *.jpg)"));

  // Load file
  if (!file_name.isEmpty()) {
    image_mat_ = cv::imread(file_name.toLatin1().data());
    cv::cvtColor(image_mat_, image_mat_, CV_BGR2RGB);
    image_mat_backup_ = image_mat_.clone();
  }

  update();
}

void ImageWidget::Save() {
  SaveAs();
}

void ImageWidget::SaveAs() {
  QString filename = QFileDialog::getSaveFileName(this,
      tr("Save Image"), ".", tr("Images(*.bmp *.png *.jpg)"));
  if (filename.isNull()) {
    return;
  }
  cv::cvtColor(image_mat_, image_mat_bgr_, CV_RGB2BGR);
  cv::imwrite(filename.toLatin1().data(), image_mat_bgr_);
}

void ImageWidget::Invert() {
  cv::MatIterator_<cv::Vec3b> iter;
  cv::MatIterator_<cv::Vec3b> iter_end;
  for (iter = image_mat_.begin<cv::Vec3b>(),
      iter_end = image_mat_.end<cv::Vec3b>(); iter != iter_end; ++iter) {
    (*iter)[0] = 255 - (*iter)[0];
    (*iter)[1] = 255 - (*iter)[1];
    (*iter)[2] = 255 - (*iter)[2];
  }

  update();
}

void ImageWidget::Mirror(bool is_horizontal, bool is_vertical) {
  cv::Mat image_mat_tmp = image_mat_.clone();
  int width = image_mat_.cols;
  int height = image_mat_.rows;

  int mode = -1;
  if (!is_horizontal && !is_vertical) {
    return;
  } else if (is_horizontal && is_vertical) {
    mode = 0;
  } else if (is_horizontal && !is_vertical) {
    mode = 1;
  } else if (!is_horizontal && is_vertical) {
    mode = 2;
  }

  for (int i = 0; i < width; ++i) {
    for (int j = 0; j < height; ++j) {
      switch (mode) {
      case 0:
        image_mat_.at<cv::Vec3b>(j, i) =
            image_mat_tmp.at<cv::Vec3b>(height - 1 - j, width - 1 - i);
        break;
      case 1:
        image_mat_.at<cv::Vec3b>(j, i) =
            image_mat_tmp.at<cv::Vec3b>(height - 1 - j, i);
        break;
      case 2:
        image_mat_.at<cv::Vec3b>(j, i) =
            image_mat_tmp.at<cv::Vec3b>(j, width - 1 - i);
        break;
      default:
        break;
      }
    }
  }

  update();
}

void ImageWidget::ToGray() {
  cv::MatIterator_<cv::Vec3b> iter;
  cv::MatIterator_<cv::Vec3b> iter_end;
  for (iter = image_mat_.begin<cv::Vec3b>(),
      iter_end = image_mat_.end<cv::Vec3b>(); iter != iter_end; ++iter) {
    int gray_val = static_cast<int>(
        ((*iter)[0] + (*iter)[1] + (*iter)[2]) / 3.f);
    (*iter)[0] = gray_val;
    (*iter)[1] = gray_val;
    (*iter)[2] = gray_val;
  }

  update();
}

void ImageWidget::Restore() {
  image_mat_ = image_mat_backup_.clone();
  update();
}

void ImageWidget::Warp() {
  image_warping_ = warping_inst_map_[warping_method_].get();
  image_warping_->SetAnchorPoints(source_points_, target_points_);
  image_warping_->WarpImage(&image_mat_);

  update();
}

void ImageWidget::ClearControlPoints() {
  source_points_.clear();
  target_points_.clear();

  update();
}

void ImageWidget::UndoSelect() {
  if (!source_points_.empty() && !target_points_.empty()) {
    source_points_.pop_back();
    target_points_.pop_back();
  }

  update();
}

void ImageWidget::SetSelectMode(bool status) {
  select_mode_ = status;
  if (!select_mode_) {
    ClearControlPoints();
  }

  update();
}

void ImageWidget::InitWarpingInstanceMap() {
  warping_inst_map_.clear();
  warping_inst_map_["IDW"].reset(new image_warping::IDWImageWarping());
  warping_inst_map_["RBF"].reset(new image_warping::RBFImageWarping());
}
