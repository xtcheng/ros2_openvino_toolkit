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
 * @brief A header file with declaration for BaseFilterSolver Class
 * @file base_filter.hpp
 */
#ifndef DYNAMIC_VINO_LIB__INFERENCES__FILTER__BASE_FILTER_HPP_
#define DYNAMIC_VINO_LIB__INFERENCES__FILTER__BASE_FILTER_HPP_

namespace dynamic_vino_lib {

/**
 * @class BaseFilterSolver
 * @brief Base class for filter solver.
 */
class BaseFilterSolver {
 public:
  BaseFilterSolver() = default;
  virtual ~BaseFilterSolver() = default;
  /**
   * @brief Checks if the given result is selected by the filter class.
   */
  template <typename T> 
  virtual bool isSelected(const T&) = 0;

  template <typename T> 
  virtual void addFilterValue(const T&) = 0;
 private:


}  // namespace dynamic_vino_lib

#endif  // DYNAMIC_VINO_LIB__INFERENCES__BASE_INFERENCE_HPP_
