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
 * @brief a header file with declaration of Pipeline Manager class
 * @file pipeline_manager.hpp
 */
#ifndef DYNAMIC_VINO_LIB__PIPELINE_MANAGER_HPP_
#define DYNAMIC_VINO_LIB__PIPELINE_MANAGER_HPP_

#include <atomic>
#include <future>
#include <memory>
#include <mutex>
#include <string>
#include <set>
#include <map>

#include <vino_param_lib/param_manager.hpp>
#include "dynamic_vino_lib/pipeline.hpp"

/**
 * @class PipelineManager
 * @brief This class manages the lifecycles of pipelines.
 */
class PipelineManager {
 public:
   /**
   * @brief Get the singleton instance of PipelineManager class.
   * The instance will be created when first call.
   * @return The reference of PipelineManager instance.
   */
  static PipelineManager& getInstance() {
    static PipelineManager manager_;
    return manager_;
  }
  
  std::shared_ptr<Pipeline> createPipeline(const Params::ParamManager::PipelineParams& params);
  void removePipeline(const std::string& name);
  PipelineManager& updatePipeline(const std::string& name, const Params::ParamManager::PipelineParams& params);
  
  enum PipelineState { PipelineState_Idle, PipelineState_Running, PipelineState_Error };
  struct PipelineData {
    Params::ParamManager::PipelineParams params;
    std::shared_ptr<Pipeline> pipeline;
    PipelineState state;
  }

 private:
  PipelineManager() {}
  PipelineManager(PipelineManager const&);
  void operator=(PipelineManager const&);

  std::map<std::string, PipelineData> pipelines_;

}

#endif // DYNAMIC_VINO_LIB__PIPELINE_MANAGER_HPP_