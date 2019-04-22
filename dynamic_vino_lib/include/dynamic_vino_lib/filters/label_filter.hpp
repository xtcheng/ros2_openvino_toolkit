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
 * @brief A header file with declaration for LabelFilterSolver Class
 * @file label_filter.hpp
 */
#ifndef DYNAMIC_VINO_LIB__FILTERS__LABEL_FILTER_HPP_
#define DYNAMIC_VINO_LIB__FILTERS__LABEL_FILTER_HPP_

#include "dynamic_vino_lib/filters/base_filter.hpp"

namespace dynamic_vino_lib {

/**
 * @class LabelFilterSolver
 * @brief Label filter solver, filtering by label information
 */
template <typename T>
class LabelFilterSolver : public BaseFilterSolver {
 public:
  LabelFilterSolver();
  virtual ~LabelFilterSolver() = default;

  bool filtering() override;

  void addFilterCondition(const std::string&);
  
  void setInference(const T*);
  
  const std::string type(){return "Label";};
 private:
  T* infer_ = nullptr;
  //std::vector<std::string> includes_;
  //std::vector<std::string> excludes_;
  std::string includes_;
  std::string excludes_;
};

}  // namespace dynamic_vino_lib

#endif  // DYNAMIC_VINO_LIB__FILTERS__LABEL_FILTER_HPP_
