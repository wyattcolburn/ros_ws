#include "onnx_model.hpp"
#include <iostream>
/*
namespace onnx_inference {

OnnxModel::OnnxModel(const std::string& model_path)
    : env_(ORT_LOGGING_LEVEL_WARNING)
{
    // Setup session options
    session_options_.SetIntraOpNumThreads(1);
    session_options_.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_EXTENDED);
    
    // Create session
    session_ = std::make_unique<Ort::Session>(env_, model_path.c_str(), session_options_);
    
    // Get model info
    Ort::AllocatorWithDefaultOptions allocator;
    
    // Setup input names
    size_t num_input_nodes = session_->GetInputCount();
    input_names_.resize(num_input_nodes);
    
    for (size_t i = 0; i < num_input_nodes; i++) {
        char* input_name = session_->GetInputName(i, allocator);
        input_names_[i] = input_name;
        
        // Get input shape
        auto type_info = session_->GetInputTypeInfo(i);
        auto tensor_info = type_info.GetTensorTypeAndShapeInfo();
        input_shape_ = tensor_info.GetShape();
        
        // Fix dynamic dimensions
        for (size_t j = 0; j < input_shape_.size(); j++) {
            if (input_shape_[j] < 0) {
                input_shape_[j] = 1; // Batch size 1
            }
        }
    }
    
    // Setup output names
    size_t num_output_nodes = session_->GetOutputCount();
    output_names_.resize(num_output_nodes);
    
    for (size_t i = 0; i < num_output_nodes; i++) {
        char* output_name = session_->GetOutputName(i, allocator);
        output_names_[i] = output_name;
        
        // Get output shape
        auto type_info = session_->GetOutputTypeInfo(i);
        auto tensor_info = type_info.GetTensorTypeAndShapeInfo();
        output_shape_ = tensor_info.GetShape();
        
        // Fix dynamic dimensions
        for (size_t j = 0; j < output_shape_.size(); j++) {
            if (output_shape_[j] < 0) {
                output_shape_[j] = 1; // Batch size 1
            }
        }
    }
}

std::vector<float> OnnxModel::run_inference(const std::vector<float>& input) {
    // Create input tensor
    auto memory_info = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
    
    Ort::Value input_tensor = Ort::Value::CreateTensor<float>(
        memory_info,
        const_cast<float*>(input.data()),
        input.size(),
        input_shape_.data(),
        input_shape_.size()
    );
    
    // Run inference
    auto output_tensors = session_->Run(
        Ort::RunOptions{nullptr},
        input_names_.data(),
        &input_tensor,
        1,
        output_names_.data(),
        output_names_.size()
    );
    
    // Get results
    float* output_data = output_tensors[0].GetTensorMutableData<float>();
    size_t output_size = 1;
    
    for (const auto& dim : output_shape_) {
        output_size *= dim;
    }
    
    return std::vector<float>(output_data, output_data + output_size);
}

} // namespace onnx_inference
*/
