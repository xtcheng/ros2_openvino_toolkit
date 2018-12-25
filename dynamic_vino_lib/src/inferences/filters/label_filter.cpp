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
 * @brief a header file with declaration of LabelFilterSolver class
 * @file label_filter.cpp
 */

#include "dynamic_vino_lib/inferences/filters/label_filter.hpp"

template <typename T>
bool dynamic_vino_lib::LabelFilterSolver::isSelected(const T& result) {
  for (const auto& l : labels_) {
    if (result.label == l) {
      return true;
    }
  }
  return false;
}

template <typename T>
void dynamic_vino_lib::LabelFilterSolver::addFilterValue(const T& val) {
  labels_.push_back(val);
}
