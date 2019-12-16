// Copyright (c) 2018-2019 Intel Corporation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

//
// @brief a header file with version information about Inference Engine.
// @file version_info.hpp
//

#ifndef DYNAMIC_VINO_LIB__UTILS__CALC_HPP_
#define DYNAMIC_VINO_LIB__UTILS__CALC_HPP_

namespace utils
{
// IntersectionOverUnion
double iou(const cv::Rect & box_1, const cv::Rect & box_2)
{
  int xmax_1 = box_1.x + box_1.width;
  int xmin_1 = box_1.x;
  int xmax_2 = box_2.x + box_2.width;
  int xmin_2 = box_2.x;

  int ymax_1 = box_1.y + box_1.height;
  int ymin_1 = box_1.y;
  int ymax_2 = box_2.y + box_2.height;
  int ymin_2 = box_2.y;

  double width_of_overlap_area = fmin(xmax_1, xmax_2) - fmax(xmin_1, xmin_2);
  double height_of_overlap_area = fmin(ymax_1, ymax_2) - fmax(ymin_1, ymin_2);
  double area_of_overlap;
  if (width_of_overlap_area < 0 || height_of_overlap_area < 0) {
    area_of_overlap = 0;
  } else {
    area_of_overlap = width_of_overlap_area * height_of_overlap_area;
  }

  double box_1_area = (ymax_1 - ymin_1) * (xmax_1 - xmin_1);
  double box_2_area = (ymax_2 - ymin_2) * (xmax_2 - xmin_2);
  double area_of_union = box_1_area + box_2_area - area_of_overlap;

  return area_of_overlap / area_of_union;
}
} //namespace utils
#endif // DYNAMIC_VINO_LIB__UTILS__CALC_HPP_