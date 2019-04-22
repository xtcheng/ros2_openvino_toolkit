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

#include "dynamic_vino_lib/filters/label_filter.hpp"
#include "dynamic_vino_lib/slog.hpp"
#include "dynamic_vino_lib/pipeline_filters.hpp"
#include "dynamic_vino_lib/inferences/object_detection.hpp"

namespace dynamic_vino_lib
{
template <typename T>
LabelFilterSolver<T>::LabelFilterSolver()
{
}

template <typename T>
void LabelFilterSolver<T>::setInference(const T* infer)
{
  if(infer != nullptr){
    infer_ = infer;
  }
}

template <typename T>
bool LabelFilterSolver<T>::filtering()
{
  if(infer_ == nullptr){
    slog::info << "there is no inference instance assigned. do nothing." << slog::endl;
    return true;
  }
  typename T::Result results = infer_->getResults();
  typename T::Result filtered_results;
  for( auto r : results){

    if(excludes_.size()>0 && excludes_.find(r.label) != std::string::npos){
      continue;
    }
    if(includes_.size()>0 && includes_.find(r.label) == std::string::npos){
      continue;
    }
    filtered_results.push_back(r);
  }
  
  infer_->setFilteredResults(filtered_results);
  return true;
}

template <typename T>
void LabelFilterSolver<T>::addFilterCondition(const std::string& cond)
{
  BaseFilterSolver::addFilterCondition(cond);
  
  std::string buff = cond;
  if(buff[0] == '='){
    buff.erase(buff.begin());
  }
  
  std::vector<std::string> conditions = PipelineFilters::split(buff, "+-");
  for (const auto& c : conditions){
    if((c[0]>='a' && c[0]<='z') || (c[0]>='A' && c[0]<='Z')){
      //includes_.push_back(c);
      includes_ += " " + c;
    } else if(c[0] == '+') {
      //includes_.push_back(c.substr(1));
      includes_ += " ";
      includes_ += c.substr(1);
    } else if(c[0] == '-') {
      //excludes_.push_back(c.substr(1));
      excludes_ += " ";
      excludes_ += c.substr(1);
    }
  }
};


template class LabelFilterSolver<ObjectDetection>;
} //namespace dynamic_vino_lib