// Copyright (c) 2018 Intel Corporation
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

/**
 * @brief a header file with declaration of ObjectDetection class and
 * ObjectDetectionResult class
 * @file object_detection.cpp
 */
#include <memory>
#include <string>
#include <vector>
#include "dynamic_vino_lib/inferences/object_detection_ssd.hpp"
#include "dynamic_vino_lib/outputs/base_output.hpp"
#include "dynamic_vino_lib/slog.hpp"

/*
// ObjectDetectionResult
dynamic_vino_lib::ObjectDetectionResult::ObjectDetectionResult(const cv::Rect & location)
: Result(location)
{
}
*/

// ObjectDetection
dynamic_vino_lib::ObjectDetection::ObjectDetection(
  bool enable_roi_constraint,
  double show_output_thresh)
: show_output_thresh_(show_output_thresh),
  enable_roi_constraint_(enable_roi_constraint), dynamic_vino_lib::ObjectDetection()
{
}

dynamic_vino_lib::ObjectDetection::~ObjectDetection() = default;

void dynamic_vino_lib::ObjectDetection::loadNetwork(
  std::shared_ptr<Models::BaseModel> network)
{
  valid_model_ = network;
  //max_proposal_count_ = network->getMaxProposalCount();
  //max_proposal_count_ = 100;

  //object_size_ = network->getObjectSize();
  setMaxBatchSize(network->getMaxBatchSize());
}

bool dynamic_vino_lib::ObjectDetection::enqueue(
  const cv::Mat & frame,
  const cv::Rect & input_frame_loc)
  {
    if (enqueued_frames_ >= max_batch_size_)
  {
    slog::warn << "Number of " << getName() << "input more than maximum("
               << max_batch_size_ << ") processed by inference" << slog::endl;
    return false;
  }

    if(valid_model_ == nullptr || engine_ == nullptr){
      return false;
    }

    if(!valid_model->enqueue(engine_, frame, input_frame_loc)){
      return false;
    }

    //nonsense!!
    //Result r(input_frame_loc);
    //results_.clear();
    //results_.emplace_back(r);
    enqueued_frames_ += 1;
    return true;
  }

bool dynamic_vino_lib::ObjectDetection::fetchResults()
{
  bool can_fetch = dynamic_vino_lib::BaseInference::fetchResults();
  if (!can_fetch) {
    return false;
  }

  results_.clear();

  return getModel()->fetchResults(getEngine(), results_);
}

const int dynamic_vino_lib::ObjectDetection::getResultsLength() const
{
  return static_cast<int>(results_.size());
}

const dynamic_vino_lib::Result * dynamic_vino_lib::ObjectDetection::getLocationResult(int idx) const
{
  return &(results_[idx]);
}

const std::string dynamic_vino_lib::ObjectDetection::getName() const
{
  return valid_model_->getModelName();
}

const void dynamic_vino_lib::ObjectDetection::observeOutput(
  const std::shared_ptr<Outputs::BaseOutput> & output)
{
  if (output != nullptr) {
    output->accept(results_);
  }
}

double dynamic_vino_lib::ObjectDetection::IntersectionOverUnion(const cv::Rect &box_1, const cv::Rect &box_2)
{
  int xmax_1 = box_1.x + box_1.width;
  int xmin_1 = box_1.x;
  int xmax_2 = box_2.x + box_2.width;
  int xmin_2 = box_2.x;

  int ymax_1 = box_1.y + box_1.height;
  int ymin_1 = box_1.y;
  int ymax_2 = box_2.y + box_2.height;
  int ymin_2 = box_2.y;

  double width_of_overlap_area = fmin(xmax_1 , xmax_2) - fmax(xmin_1, xmin_2);
  double height_of_overlap_area = fmin(ymax_1, ymax_2) - fmax(ymin_1, ymin_2);
  double area_of_overlap;
  if (width_of_overlap_area < 0 || height_of_overlap_area < 0)
    area_of_overlap = 0;
  else
    area_of_overlap = width_of_overlap_area * height_of_overlap_area;

 double box_1_area = (ymax_1 - ymin_1)  * (xmax_1 - xmin_1);
 double box_2_area = (ymax_2 - ymin_2)  * (xmax_2 - xmin_2);
 double area_of_union = box_1_area + box_2_area - area_of_overlap;

 return area_of_overlap / area_of_union;
}