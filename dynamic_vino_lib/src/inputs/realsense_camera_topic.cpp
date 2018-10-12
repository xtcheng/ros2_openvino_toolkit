/*
 * Copyright (c) 2018 Intel Corporation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/**
 * @brief a header file with declaration of RealSenseCamera class
 * @file realsense_camera_topic.cpp
 */

#include "dynamic_vino_lib/inputs/realsense_camera_topic.hpp"
#include "dynamic_vino_lib/slog.hpp"

#include <cv_bridge/cv_bridge.h>

#define INPUT_TOPIC "/camera/color/image_raw"

Input::RealSenseCameraTopic::RealSenseCameraTopic() : Node("realsense_topic") {

}

bool Input::RealSenseCameraTopic::initialize() {
  slog::info << "before cameraTOpic init" << slog::endl;
  std::shared_ptr<rclcpp::Node> node(this);
  setHandler(node);
  sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/camera/color/image_raw",
      std::bind(&RealSenseCameraTopic::cb, this, std::placeholders::_1));

  read_counter_ = 0;
  store_counter_ = 0;
  image_count_ = 0;
  images_.resize(kBufferedImageNum);
  return true;
}

void Input::RealSenseCameraTopic::cb(const sensor_msgs::msg::Image::SharedPtr image_msg) {
  //slog::info << "Receiving a new image from Camera topic." << slog::endl;
  if (image_count_ < kBufferedImageNum ){
    images_[store_counter_] = cv_bridge::toCvCopy(image_msg, "bgr8")->image;
    store_counter_ = (store_counter_+1) % kBufferedImageNum;
    image_count_++;
  }
}

bool Input::RealSenseCameraTopic::read(cv::Mat* frame) {
  
  if (image_count_ <= 0 || images_[read_counter_].empty()){
    slog::warn << "No data received in CameraTopic instance" << slog::endl;
    return false;
  }

  int i = image_count_ <= 0? (read_counter_+kBufferedImageNum-1)%kBufferedImageNum : read_counter_;
  *frame = images_[i];

  if (image_count_ > 0) {
    read_counter_ = (read_counter_ + 1) % kBufferedImageNum;
    image_count_--;
  }
  return true;
}

void Input::RealSenseCameraTopic::config() {
  // TODO(weizhi): config
}
