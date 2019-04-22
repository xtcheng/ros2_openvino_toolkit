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
 * @brief a header file with declaration of PipelineFilters class
 * @file pipeline_filters.cpp
 */

#include <memory>
#include <string>
#include <utility>

#include <vino_param_lib/param_manager.hpp>
#include "dynamic_vino_lib/pipeline_filters.hpp"


PipelineFilters::PipelineFilters() {}

PipelineFilters::PipelineFilters(
    const Params::ParamManager::PipelineRawData& params) {
  add(params.filters);
}

void PipelineFilters::add(
    const Params::ParamManager::FilterRawData& filters) {
  
  for( auto& f : filters)
  {
    
    add(f.input, f.output, f.filters);
  }
}

void PipelineFilters::add(
    const std::string& input, const std::string& output, const std::string& filter) {

  PipelineFilterNative filter_native;
  filter_native.input = input;
  filter_native.output = output;
  
  std::vector<std::string> filter_conditions = split(filter, ",");
  
  for(auto& fc : filter_conditions)
  {
    std::vector<std::string> type_and_values = split(fc, "=><");
    if (type_and_values.size() != 2)
    {
      slog::err << "ERROR in filter conditions, the correct one should be A=B+C" << slog::endl;
      //throw
    }
    std::shared_ptr<BaseFilterSolver> solver = getFilterSolver(input, type_and_values[0]);
    solver->addFilterCondition(type_and_values[1]);
    pipeline_filters_.filter_solvers.push_back(solver);
  }

}
#define _GEN_FILTER_SOLVER_(infer, name) {std::make_shared<##name##FilterSolver<##infer##>()}
#define GEN_FILTER_SOLVER(infer, name) _GEN_FILTER_SOLVER_(infer, name)

std::shared_ptr<BaseFilterSolver>
PipelineFilters::getFilterSolver(const std::string infer, const std::string& type)
{
  if(!supportedType(type)){
    return nullptr;
  }
  
  std::shared_ptr<BaseFilterSolver> solver = nullptr;
  solver = GEN_FILTER_SOLVER(infer.c_str(), type.c_str());
  
  return solver;

}

std::vector<std::shared_ptr<BaseFilterSolver>>
PipelineFilters::findPipelineFilters(const std::string& input, const std::string& output)
{
  std::vector<std::shared_ptr<BaseFilterSolver>> ret;
  for(const auto& s : pipeline_filters_.filter_solvers)
  {
    if(s->input == input && s->output==output)
    {
      ret.push_back(s);
    }
  }
  
  return ret;
}

std::vector<std::string> PipelineFilters::split(const std::string& source, const std::string& splitter)
{
  std::string buff{""};
  std::vector<std::string> ret;
  
  for(auto s : splitter)
  {
    buff.clear();
    for(auto b : source)
    {
      
      if(b == s && buff != "")
      {
	ret.push_back(buff);
	buff.clear();
      } else {
	buff += b;
      }
    }
    if(buff != "")
    {
      ret.push_back(buff);
    }
  }
  
  return ret;
}
