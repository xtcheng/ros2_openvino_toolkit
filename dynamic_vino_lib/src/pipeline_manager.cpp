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
 * @file pipeline_manager.cpp
 */

#include <utility>
#include <memory>
#include <string>

#include "dynamic_vino_lib/pipeline.hpp"
#include <vino_param_lib/param_manager.hpp>
#include "dynamic_vino_lib/pipeline_params.hpp"

std::shared_ptr<Pipeline>
PipelineManager::createPipeline(const Params::ParamManager::PipelineParams& params) {
  if (params.name == ""){
    throw std::logic_error("The name of pipeline won't be empty!");
  }
  
  std::shared_ptr<Pipeline> pipeline = std::make_shared<Pipeline>(params.name);
  pipeline->getParameters()->update(params);
  
  auto inputs = parseInputDevice(params);
  if (inputs.size() != 1 ) {
    slog::err << "currently one pipeline only supports ONE input." << slog::endl;
    return nullptr;
  }
  for (auto it=inputs.begin(); it != inputs.end(); ++it) {
    pipeline->add(it->first, it->second);
  }
  
  
  
  
  
  
}

std::map<std::string, std::shared_ptr<Input::BaseInputDevice> >
PipelineManager::parseInputDevice(const Params::ParamManager::PipelineParams& params) {

  std::map<std::string, std::shared_ptr<Input::BaseInputDevice> > inputs;
  for (auto& name : params.inputs) {
    std::cout << "Parsing InputDvice: " << d << std::endl;
    std::shared_ptr<Input::BaseInputDevice> device = nullptr;
    if (name == PipelineParams::kInputType_RealSenseCamera) {
      device = std::make_unique<Input::RealSenseCamera>();
    } else if (name == PipelineParams::kInputType_StandardCamera) {
      device = std::make_unique<Input::StandardCamera>();
    } else if (name == PipelineParams::kInputType_CameraTopic) {
      device = std::make_unique<Input::RealSenseCameraTopic>();
    } else if (name == PipelineParams::kInputType_Video) {
      if (params.input_meta != "") {
        device = std::make_unique<Input::Video>(params.input_meta);
      }
    } else if (name == PipelineParams::kInputType_Image) {
      if (params.input_meta != "") {
        device = std::make_unique<Input::Image>(params.input_meta);
      }
    }
    
    if (device != nullptr) {
      inputs.insert({name, device});
    }
  }
  
  return inputs;
}

std::map<std::string, std::shared_ptr<Outputs::BaseOutput> >
PipelineManager::parseInputDevice(const Params::ParamManager::PipelineParams& params) {

  std::map<std::string, std::shared_ptr<Outputs::BaseOutput> > outputs;
  for (auto& name : params.outputs) {
    std::cout << "Parsing Output: " << name << std::endl;
    std::shared_ptr<Outputs::BaseOutput> object = nullptr;
    if (name == PipelineParams::kInputType_RealSenseCamera) {
      device = std::make_unique<Input::RealSenseCamera>();
    } else if (name == PipelineParams::kInputType_StandardCamera) {
      device = std::make_unique<Input::StandardCamera>();
    } else if (name == PipelineParams::kInputType_CameraTopic) {
      device = std::make_unique<Input::RealSenseCameraTopic>();
    } else if (name == PipelineParams::kInputType_Video) {
      if (params.input_meta != "") {
        device = std::make_unique<Input::Video>(params.input_meta);
      }
    } else if (name == PipelineParams::kInputType_Image) {
      if (params.input_meta != "") {
        device = std::make_unique<Input::Image>(params.input_meta);
      }
    }
    
    if (device != nullptr) {
      outputs.insert({d, device});
    }
  }
  
  return inputs;
}