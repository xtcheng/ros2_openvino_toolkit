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
#include "dynamic_vino_lib/pipeline_manager.hpp"
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
  
  auto outputs = parseOutput(params);
  for (auto it=outputs.begin(); it != outputs.end(); ++it) {
    pipeline->add(it->first, it->second);
  }
  
  auto infers = parseInference(params);
  for (auto it=infers.begin(); it != infers.end(); ++it) {
    pipeline->add(it->first, it->second);
  }
  
  for (auto it=params.connects.begin(); it != params.connects.end(); ++it) {
    pipeline->add(it->first, it->second);
  }
  
  PipelineData data;
  data.pipeline = pipeline;
  data.params = params;
  data.state = PipelineState_Idle;

  return pipeline;
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
PipelineManager::parseOutput(const Params::ParamManager::PipelineParams& params) {

  std::map<std::string, std::shared_ptr<Outputs::BaseOutput> > outputs;
  for (auto& name : params.outputs) {
    std::cout << "Parsing Output: " << name << std::endl;
    std::shared_ptr<Outputs::BaseOutput> object = nullptr;
    if (name == PipelineParams::kOutputTpye_RViz || 
      name == PipelineParams::kOutputTpye_RosTopic) {
      object = std::make_shared<Outputs::RosTopicOutput>();
    } else if (name == PipelineParams::kOutputTpye_ImageWindow) {
      object = std::make_shared<Outputs::ImageWindowOutput>("Results");
    }
    
    if (object != nullptr) {
      outputs.insert({name, device});
    }
  }

  return outputs;
}

std::map<std::string, std::shared_ptr<dynamic_vino_lib::BaseInference>>
PipelineManager::parseInference(const Params::ParamManager::PipelineParams& params) {

  /**< update plugins for devices >**/
  auto pcommon = Params::ParamManager::getInstance().getCommon();
  std::string FLAGS_l = pcommon.custom_cpu_library;
  std::string FLAGS_c = pcommon.custom_cldnn_library;
  bool FLAGS_pc = pcommon.enable_performance_count;
  
  std::map<std::string, std::shared_ptr<dynamic_vino_lib::BaseInference>> inferences;
  for (auto& infer : params.infers) {

    if(infer.name.empty() || infer.model.empty()) {
      continue;
    }
    std::cout << "Parsing Inference: " << infer << std::endl;
    std::shared_ptr<dynamic_vino_lib::BaseInference> object = nullptr;
    if (plugins_for_devices_.find(infer.engine) == plugins_for_devices_.end()) {
      plugins_for_devices_[infer.engine] =
          *Factory::makePluginByName(infer.engine, FLAGS_l, FLAGS_c, FLAGS_pc);
    }
      
    if (infer.name == PipelineParams::kInferTpye_FaceDetection) {
      object = createFaceDetection(infer);
      
    } else if (name == PipelineParams::kInferTpye_AgeGenderRecognition) {
      object = createAgeGenderRecognition(infer);
      
    } else if (name == PipelineParams::kInferTpye_EmotionRecognition) {
      object = createEmotionRecognition(infer);
      
    } else if (name == PipelineParams::kInferTpye_HeadPoseEstimation) {
      object = createHeadPoseEstimation(infer);
      
    } else if (name == PipelineParams::kInferTpye_ObjectDetection) {
      object = createObjectDetection(infer);
    }
    
    if (object != nullptr) {
      inferences.insert({infer.name, object});
    }
  }

  return inferences;
}

std::shared_ptr<dynamic_vino_lib::BaseInference>
PipelineManager::createFaceDetection(const Params::ParamManager::InferenceParams& infer) {
  //TODO: add batch size in param_manager
  auto face_detection_model =
        std::make_shared<Models::FaceDetectionModel>(infer.model, 1, 1, 1);
  face_detection_model->modelInit();
  auto face_detection_engine = std::make_shared<Engines::Engine>(
        plugins_for_devices_[infer.engine], face_detection_model);
  auto face_inference_ptr =
        std::make_shared<dynamic_vino_lib::FaceDetection>(0.5);//TODO: add output_threshold in param_manager
  face_inference_ptr->loadNetwork(face_detection_model);
  face_inference_ptr->loadEngine(face_detection_engine);
  
  return face_inference_ptr;
}

std::shared_ptr<dynamic_vino_lib::BaseInference>
PipelineManager::createAgeGenderRecognition(const Params::ParamManager::InferenceParams& infer) {
  auto model =
        std::make_shared<Models::AgeGenderDetectionModel>(infer.model, 1, 2, 16);
  model->modelInit();
  auto engine = std::make_shared<Engines::Engine>(
        plugins_for_devices_[infer.engine], model);
  auto infer =
        std::make_shared<dynamic_vino_lib::AgeGenderDetection>();
  infer->loadNetwork(model);
  infer->loadEngine(engine);
  
  return infer;
}

std::shared_ptr<dynamic_vino_lib::BaseInference>
PipelineManager::createEmotionRecognition(const Params::ParamManager::InferenceParams& infer) {
  auto model =
        std::make_shared<Models::EmotionDetectionModel>(infer.model, 1, 1, 16);
  model->modelInit();
  auto engine = std::make_shared<Engines::Engine>(
        plugins_for_devices_[infer.engine], model);
  auto infer =
        std::make_shared<dynamic_vino_lib::EmotionDetection>();
  infer->loadNetwork(model);
  infer->loadEngine(engine);
  
  return infer;
}

std::shared_ptr<dynamic_vino_lib::BaseInference>
PipelineManager::createHeadPoseEstimation(const Params::ParamManager::InferenceParams& infer) {
  auto model =
        std::make_shared<Models::HeadPoseDetectionModel>(infer.model, 1, 3, 16);
  model->modelInit();
  auto engine = std::make_shared<Engines::Engine>(
        plugins_for_devices_[infer.engine], model);
  auto infer =
        std::make_shared<dynamic_vino_lib::HeadPoseDetection>();
  infer->loadNetwork(model);
  infer->loadEngine(engine);

  return infer;
}

std::shared_ptr<dynamic_vino_lib::BaseInference>
PipelineManager::createObjectDetection(const Params::ParamManager::InferenceParams& infer) {
  //TODO: not implemented yet
  
  
  return createFaceDetection(infer);
}