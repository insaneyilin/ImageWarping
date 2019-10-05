// Copyright (c) 2019 Yilin Gui. All rights reserved.
//
// filename: scanline.cpp

#include "image_warping/scanline.h"

#include <cmath>
#include <algorithm>
#include <iostream>

using Edge = image_warping::ScanLineAlgorithm::Edge;

namespace image_warping {

int ScanLineAlgorithm::s_y_min_ = 1e9;
int ScanLineAlgorithm::s_y_max_ = -1e9;
std::vector<std::vector<Edge> > ScanLineAlgorithm::s_et_;
std::vector<Edge> ScanLineAlgorithm::s_aet_;

void ScanLineAlgorithm::GetInternalPointsOfPolygon(
    const std::vector<Eigen::Vector2f> &polygon,
    std::vector<Eigen::Vector2i> *points) {
  points->clear();
  s_y_min_ = 1e9;
  s_y_max_ = -1e9;
  s_et_.clear();
  s_aet_.clear();

  const int num_poly_pts = polygon.size();
  for (int i = 0; i < num_poly_pts; ++i) {
    const auto &pt = polygon[i];
    int x1 = pt[0];
    int y1 = pt[1];
    s_y_min_ = std::min(y1, s_y_min_);
    s_y_max_ = std::max(y1, s_y_max_);
  }
  s_et_.resize(s_y_max_ - s_y_min_ + 1);
  for (int i = 0; i < num_poly_pts; ++i) {
    const auto &pt = polygon[i];
    int x1 = pt[0];
    int y1 = pt[1];
    const auto &next_pt = polygon[(i + 1) % num_poly_pts];
    int x2 = next_pt[0];
    int y2 = next_pt[1];

    AddEdge(x1, y1, x2, y2);
  }
  // scan lines
  for (int i = 0; i < s_y_max_ - s_y_min_ + 1; ++i) {
    for (int j = 0; j < s_et_[i].size(); ++j) {
      s_aet_.push_back(s_et_[i][j]);
    }
    std::sort(s_aet_.begin(), s_aet_.end(),
        [](const Edge &e1, const Edge &e2) {
      if (std::fabs(e1.x - e2.x) < 1e-6) {
        return e1.k < e2.k;
      } else {
        return e1.x < e2.x;
      }
    });
    const int aet_size = s_aet_.size();
    for (int j = 0; j + 1 < aet_size; j += 2) {
      int l_start = (s_aet_[j].x - static_cast<int>(s_aet_[j].x > 0.5f) ?
          static_cast<int>(s_aet_[j].x) + 1 : static_cast<int>(s_aet_[j].x));
      int l_end = (s_aet_[j + 1].x - static_cast<int>(s_aet_[j + 1].x > 0.5f) ?
          static_cast<int>(s_aet_[j + 1].x) + 1 :
              static_cast<int>(s_aet_[j + 1].x));
      for (int l = l_start; l <= l_end; ++l) {
        Eigen::Vector2i pti;
        pti[0] = l;
        pti[1] = i + s_y_min_;
        points->push_back(pti);
      }
    }
    for (auto itr = s_aet_.begin(); itr != s_aet_.end();) {
      if (itr->ymax <= i + s_y_min_) {
        s_aet_.erase(itr);
      } else {
        ++itr;
      }
    }
    // incrementally update
    for (auto &e : s_aet_) {
      e.x += e.k;
    }
  }
}

void ScanLineAlgorithm::AddEdge(int x1, int y1, int x2, int y2) {
  Edge edge;
  // ignore horizontal lines
  if (y1 < y2) {
    edge.ymax = y2 - 1;
    edge.x = x1;
    edge.k = 1.f * (x2 - x1) / (y2 - y1);
    s_et_[y1 - s_y_min_].push_back(edge);
  } else if (y1 > y2) {
    edge.ymax = y1 - 1;
    edge.x = x2;
    edge.k = 1.f * (x2 - x1) / (y2 - y1);
    s_et_[y2 - s_y_min_].push_back(edge);
  }
}

}  // namespace image_warping
