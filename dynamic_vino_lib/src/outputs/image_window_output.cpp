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
 * @brief a header file with declaration of ImageWindowOutput class
 * @file image_window_output.cpp
 */

#include <algorithm>
#include <string>
#include <vector>
#include <map>
#include <iomanip>

#include "dynamic_vino_lib/outputs/image_window_output.hpp"
#include "dynamic_vino_lib/pipeline.hpp"
#include "dynamic_vino_lib/utils/calc.hpp"

Outputs::ImageWindowOutput::ImageWindowOutput(const std::string & output_name, int focal_length)
: BaseOutput(output_name), focal_length_(focal_length)
{
  cv::namedWindow(output_name_, cv::WINDOW_AUTOSIZE);
}

void Outputs::ImageWindowOutput::feedFrame(const cv::Mat & frame)
{
  // frame_ = frame;
  frame_ = frame.clone();
  if (camera_matrix_.empty()) {
    int cx = frame.cols / 2;
    int cy = frame.rows / 2;
    camera_matrix_ = cv::Mat::zeros(3, 3, CV_32F);
    camera_matrix_.at<float>(0) = focal_length_;
    camera_matrix_.at<float>(2) = static_cast<float>(cx);
    camera_matrix_.at<float>(4) = focal_length_;
    camera_matrix_.at<float>(5) = static_cast<float>(cy);
    camera_matrix_.at<float>(8) = 1;
  }
}

unsigned Outputs::ImageWindowOutput::findOutput(
  const cv::Rect & result_rect)
{
  for (unsigned i = 0; i < outputs_.size(); i++) {
    if (outputs_[i].rect == result_rect) {
      return i;
    }
  }

  OutputData output;
  output.desc = "";
  output.scalar = cv::Scalar(255, 0, 0);
  outputs_.push_back(output);
  return outputs_.size() - 1;
}

int Outputs::ImageWindowOutput::findOutputInRegion(
  const cv::Rect & result_rect)
{
  for (int i = 0; i < outputs_.size(); i++) {
    if((result_rect & outputs_[i].rect) == result_rect) {
      return i;
    }
  }

  return -1;
}

void Outputs::ImageWindowOutput::accept(
  const std::vector<dynamic_vino_lib::LicensePlateDetectionResult> & results)
{
  for (unsigned i = 0; i < results.size(); i++) {
    cv::Rect result_rect = results[i].getLocation();
    unsigned target_index = findOutput(result_rect);
    outputs_[target_index].rect = result_rect;
    outputs_[target_index].desc += ("[" + results[i].getLicense() + "]");
  }
}

void Outputs::ImageWindowOutput::accept(
  const std::vector<dynamic_vino_lib::VehicleAttribsDetectionResult> & results)
{
  for (unsigned i = 0; i < results.size(); i++) {
    cv::Rect result_rect = results[i].getLocation();
    unsigned target_index = findOutput(result_rect);
    outputs_[target_index].rect = result_rect;
    outputs_[target_index].desc +=
      ("[" + results[i].getColor() + "," + results[i].getType() + "]");
  }
}

void Outputs::ImageWindowOutput::accept(
  const std::vector<dynamic_vino_lib::FaceReidentificationResult> & results)
{
  for (unsigned i = 0; i < results.size(); i++) {
    cv::Rect result_rect = results[i].getLocation();
    unsigned target_index = findOutput(result_rect);
    outputs_[target_index].rect = result_rect;
    std::string id = results[i].getFaceID();
#ifdef PERSON_ANALYSIS
    id = getTunedPersonID(result_rect, results[i].getFaceID());
    outputs_[target_index].person.id = id;
    outputs_[target_index].person.roi = result_rect;
#endif
    outputs_[target_index].desc += "[" + id + "]";
  }
}

void Outputs::ImageWindowOutput::accept(
  const std::vector<dynamic_vino_lib::LandmarksDetectionResult> & results)
{
  for (unsigned i = 0; i < results.size(); i++) {
    cv::Rect result_rect = results[i].getLocation();
    unsigned target_index = findOutput(result_rect);
    std::vector<cv::Point> landmark_points = results[i].getLandmarks();
    for (int j = 0; j < landmark_points.size(); j++) {
      outputs_[target_index].landmarks.push_back(landmark_points[j]);
    }
  }
}

void Outputs::ImageWindowOutput::accept(
  const std::vector<dynamic_vino_lib::PersonAttribsDetectionResult> & results)
{
  for (unsigned i = 0; i < results.size(); i++) {
    cv::Rect result_rect = results[i].getLocation();
    unsigned target_index = findOutput(result_rect);
    if (results[i].getMaleProbability() < 0.5) {
      outputs_[target_index].scalar = cv::Scalar(0, 0, 255);
    }
    outputs_[target_index].rect = result_rect;
    outputs_[target_index].desc += "[" + results[i].getAttributes() + "]";
  }
}

void Outputs::ImageWindowOutput::accept(
  const std::vector<dynamic_vino_lib::PersonReidentificationResult> & results)
{
  for (unsigned i = 0; i < results.size(); i++) {
    cv::Rect result_rect = results[i].getLocation();
    unsigned target_index = findOutput(result_rect);
    outputs_[target_index].rect = result_rect;
    //outputs_[target_index].desc += "[" + results[i].getPersonID() + "]";
    std::string id = results[i].getPersonID();
#ifdef PERSON_ANALYSIS
    id = getTunedPersonID(result_rect, results[i].getPersonID());
    outputs_[target_index].person.id = id;
    outputs_[target_index].person.roi = result_rect;
    outputs_[target_index].person.age = 0;
    outputs_[target_index].person.isMale = true;
#endif
    outputs_[target_index].desc += "[" + id + "]";
  }
}

void Outputs::ImageWindowOutput::mergeMask(
  const std::vector<dynamic_vino_lib::ObjectSegmentationResult> & results)
{
  std::map<std::string, int> class_color;
  for (unsigned i = 0; i < results.size(); i++) {
    std::string class_label = results[i].getLabel();
    if (class_color.find(class_label) == class_color.end()) {
      class_color[class_label] = class_color.size();
    }
    auto & color = colors_[class_color[class_label]];
    const float alpha = 0.7f;
    const float MASK_THRESHOLD = 0.5;

    cv::Rect location = results[i].getLocation();
    cv::Mat roi_img = frame_(location);
    cv::Mat mask = results[i].getMask();
    cv::Mat colored_mask(location.height, location.width, frame_.type());

    for (int h = 0; h < mask.size().height; ++h) {
      for (int w = 0; w < mask.size().width; ++w) {
        for (int ch = 0; ch < colored_mask.channels(); ++ch) {
          colored_mask.at<cv::Vec3b>(h, w)[ch] = mask.at<float>(h, w) > MASK_THRESHOLD ?
            255 * color[ch] :
            roi_img.at<cv::Vec3b>(h, w)[ch];
        }
      }
    }
    cv::addWeighted(colored_mask, alpha, roi_img, 1.0f - alpha, 0.0f, roi_img);
  }
}

void Outputs::ImageWindowOutput::accept(
  const std::vector<dynamic_vino_lib::ObjectSegmentationResult> & results)
{
  for (unsigned i = 0; i < results.size(); i++) {
    cv::Rect result_rect = results[i].getLocation();
    unsigned target_index = findOutput(result_rect);
    outputs_[target_index].rect = result_rect;
    auto fd_conf = results[i].getConfidence();
    if (fd_conf >= 0) {
      std::ostringstream ostream;
      ostream << "[" << std::fixed << std::setprecision(3) << fd_conf << "]";
      outputs_[target_index].desc += ostream.str();
    }
    auto label = results[i].getLabel();
    outputs_[target_index].desc += "[" + label + "]";
  }
  mergeMask(results);
}

void Outputs::ImageWindowOutput::accept(
  const std::vector<dynamic_vino_lib::FaceDetectionResult> & results)
{
  for (unsigned i = 0; i < results.size(); i++) {
    cv::Rect result_rect = results[i].getLocation();
    unsigned target_index = findOutput(result_rect);
    outputs_[target_index].rect = result_rect;
    auto fd_conf = results[i].getConfidence();
    if (fd_conf >= 0) {
      std::ostringstream ostream;
      ostream << "[" << std::fixed << std::setprecision(3) << fd_conf << "]";
      outputs_[target_index].desc += ostream.str();
    }
  }
}

void Outputs::ImageWindowOutput::accept(
  const std::vector<dynamic_vino_lib::ObjectDetectionResult> & results)
{
  for (unsigned i = 0; i < results.size(); i++) {
    cv::Rect result_rect = results[i].getLocation();
    unsigned target_index = findOutput(result_rect);
    outputs_[target_index].rect = result_rect;
    auto fd_conf = results[i].getConfidence();
    if (fd_conf >= 0) {
      std::ostringstream ostream;
      ostream << "[" << std::fixed << std::setprecision(3) << fd_conf << "]";
      outputs_[target_index].desc += ostream.str();
    }
    auto label = results[i].getLabel();
    outputs_[target_index].desc += "[" + label + "]";
  }
}

void Outputs::ImageWindowOutput::accept(
  const std::vector<dynamic_vino_lib::EmotionsResult> & results)
{
  for (unsigned i = 0; i < results.size(); i++) {
    cv::Rect result_rect = results[i].getLocation();
    unsigned target_index = findOutput(result_rect);
    outputs_[target_index].rect = result_rect;
    std::ostringstream ostream;
    ostream << "[" << results[i].getLabel() << "]";
    outputs_[target_index].desc += ostream.str();
  }
}

void Outputs::ImageWindowOutput::accept(
  const std::vector<dynamic_vino_lib::AgeGenderResult> & results)
{
  for (unsigned i = 0; i < results.size(); i++) {
    cv::Rect result_rect = results[i].getLocation();
    unsigned target_index = findOutput(result_rect);
    outputs_[target_index].rect = result_rect;
    std::ostringstream ostream;
    ostream << "[Y" << std::fixed << std::setprecision(0) << results[i].getAge() << "]";
    outputs_[target_index].desc += ostream.str();
    auto male_prob = results[i].getMaleProbability();
#ifdef PERSON_ANALYSIS
    int person_index = findOutputInRegion(result_rect);
    outputs_[target_index].person.age = (int)(results[i].getAge());
    //slog::info <<target_index<< ":age=" << outputs_[target_index].person.age << slog::endl;
    outputs_[target_index].person.isMale = true;
    if(person_index >= 0) {
      outputs_[person_index].person.age = (int)(results[i].getAge());
      outputs_[person_index].person.isMale = true;
    }
#endif
    if (male_prob < 0.5) {
      outputs_[target_index].scalar = cv::Scalar(0, 0, 255);
#ifdef PERSON_ANALYSIS
      outputs_[target_index].person.isMale = false;
      if(person_index >= 0) {
        outputs_[person_index].person.isMale = false;
      }
#endif
    }
  }
}

cv::Point Outputs::ImageWindowOutput::calcAxis(
  cv::Mat r, double cx, double cy, double cz,
  cv::Point cp)
{
  cv::Mat Axis(3, 1, CV_32F);
  Axis.at<float>(0) = cx;
  Axis.at<float>(1) = cy;
  Axis.at<float>(2) = cz;
  cv::Mat o(3, 1, CV_32F, cv::Scalar(0));
  o.at<float>(2) = camera_matrix_.at<float>(0);
  Axis = r * Axis + o;
  cv::Point point;
  point.x = static_cast<int>((Axis.at<float>(0) / Axis.at<float>(2) * camera_matrix_.at<float>(0)) +
    cp.x);
  point.y = static_cast<int>((Axis.at<float>(1) / Axis.at<float>(2) * camera_matrix_.at<float>(4)) +
    cp.y);
  return point;
}

cv::Mat Outputs::ImageWindowOutput::getRotationTransform(double yaw, double pitch, double roll)
{
  pitch *= CV_PI / 180.0;
  yaw *= CV_PI / 180.0;
  roll *= CV_PI / 180.0;
  cv::Matx33f Rx(1, 0, 0, 0, cos(pitch), -sin(pitch), 0, sin(pitch), cos(pitch));
  cv::Matx33f Ry(cos(yaw), 0, -sin(yaw), 0, 1, 0, sin(yaw), 0, cos(yaw));
  cv::Matx33f Rz(cos(roll), -sin(roll), 0, sin(roll), cos(roll), 0, 0, 0, 1);
  auto r = cv::Mat(Rz * Ry * Rx);
  return r;
}

void Outputs::ImageWindowOutput::accept(
  const std::vector<dynamic_vino_lib::HeadPoseResult> & results)
{
  for (unsigned i = 0; i < results.size(); i++) {
    auto result = results[i];
    cv::Rect result_rect = result.getLocation();
    unsigned target_index = findOutput(result_rect);
    outputs_[target_index].rect = result_rect;
    double yaw = result.getAngleY();
    double pitch = result.getAngleP();
    double roll = result.getAngleR();
    double scale = 50;
    feedFrame(frame_);
    cv::Mat r = getRotationTransform(yaw, pitch, roll);
    cv::Rect location = result.getLocation();
    auto cp = cv::Point(location.x + location.width / 2, location.y + location.height / 2);
    outputs_[target_index].hp_cp = cp;
    outputs_[target_index].hp_x = calcAxis(r, scale, 0, 0, cp);
    outputs_[target_index].hp_y = calcAxis(r, 0, -scale, 0, cp);
    outputs_[target_index].hp_ze = calcAxis(r, 0, 0, -scale, cp);
    outputs_[target_index].hp_zs = calcAxis(r, 0, 0, scale, cp);
  }
}

void Outputs::ImageWindowOutput::decorateFrame()
{
  if (getPipeline()->getParameters()->isGetFps()) {
    int fps = getPipeline()->getFPS();
    std::stringstream ss;
    ss << "FPS: " << fps;
    cv::putText(frame_, ss.str(), cv::Point2f(0, 65), cv::FONT_HERSHEY_TRIPLEX, 0.5,
      cv::Scalar(255, 0, 0));
  }

  for (auto o : outputs_) {
    auto new_y = std::max(15, o.rect.y - 15);
    cv::putText(frame_, o.desc, cv::Point2f(o.rect.x, new_y), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8,
      o.scalar);
    cv::rectangle(frame_, o.rect, o.scalar, 1);
    if (o.hp_cp != o.hp_x) {
      cv::line(frame_, o.hp_cp, o.hp_x, cv::Scalar(0, 0, 255), 2);
    }
    if (o.hp_cp != o.hp_y) {
      cv::line(frame_, o.hp_cp, o.hp_y, cv::Scalar(0, 255, 0), 2);
    }
    if (o.hp_zs != o.hp_ze) {
      cv::line(frame_, o.hp_zs, o.hp_ze, cv::Scalar(255, 0, 0), 2);
      cv::circle(frame_, o.hp_ze, 3, cv::Scalar(255, 0, 0), 2);
    }
    for (int i = 0; i < o.landmarks.size(); i++) {
      cv::circle(frame_, o.landmarks[i], 3, cv::Scalar(255, 0, 0), 2);
    }

#ifdef PERSON_ANALYSIS
    updatePersonDB(o.person);
#endif
  }
#ifdef PERSON_ANALYSIS
    decoratePersonAnalysis();
#endif
  outputs_.clear();
}

#ifdef PERSON_ANALYSIS
void Outputs::ImageWindowOutput::updatePersonDB(const struct Outputs::ImageWindowOutput::Person & person)
{
  bool found = false;
  if(person.id.empty()) {
    return;
  }
  for (auto & p : persons_) {
    if(person.id == p.id) {
      if (p.age < 10) p.age = person.age;
      if (person.age > 10)
        p.age = p.age * 0.9 + person.age * 0.1;
        p.isMale = person.isMale;
      found = true;
    }
  }

  if(!found) {
    persons_.push_back(person);
  }
}

std::string Outputs::ImageWindowOutput::getTunedPersonID(const cv::Rect & box, const std::string default_id)
{
#if 0
  double max_iou = 0.0;
  int index = -1;
  for(size_t i=0; i<persons_.size(); i++) {
    auto iou = utils::iou(box, persons_[i].roi);
    if (iou > max_iou) {
      max_iou = iou;
      index = i;
    }
  }
  if(max_iou > 0.5) {
    return persons_[index].id;
  }
#endif
  return default_id;
}
void Outputs::ImageWindowOutput::decoratePersonAnalysis()
{
  auto scalar = cv::Scalar(255, 0, 255);
  std::string person_number = "Persons: " + std::to_string(persons_.size());
  cv::putText(frame_, person_number, cv::Point2f(0, 85), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, scalar);
  std::map<int, int> age_state;
  for(auto p : persons_){
    int age = (p.age / 10) * 10;
    //slog::info << "Decorating: age=" << age <<"(should be:" << p.age <<slog::endl;
    if (age <= 0 || age > 100) continue;
    if (age_state.find(age) != age_state.end()){
      age_state[age] = age_state[age] + 1;
    } else {
      age_state[age] = 1;
    }
    
  }
  std::string age_string = "Ages:";
  for (auto it=age_state.begin(); it!=age_state.end(); ++it) {
    age_string += " " + std::to_string(it->first) + "s=" + std::to_string(it->second);
  }
  cv::putText(frame_, age_string, cv::Point2f(0, 105), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, scalar);

  int males = 0;
  for(auto p : persons_) {
    if(p.isMale) {
      males++;
    }
  }
  std::string gender = "Gender: Males=" + std::to_string(males) + ", Females=" + 
    std::to_string(persons_.size()-males);
  cv::putText(frame_, gender, cv::Point2f(0, 125), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, scalar);
  int y = 140;
  for(auto p : persons_){
    std::string person;
    person += p.id;
    person += ":" + std::to_string(p.age);
    person += (p.isMale?"M":"F");
    cv::putText(frame_, person, cv::Point2f(0, y), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, scalar);
    y += 15;
  }
}
#endif

void Outputs::ImageWindowOutput::handleOutput()
{
  if(frame_.cols == 0 || frame_.rows == 0){
    return;
  }
  decorateFrame();
  cv::imshow(output_name_, frame_);
  cv::waitKey(1);
}
