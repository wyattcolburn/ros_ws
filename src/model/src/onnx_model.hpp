#ifndef ONNX_INFERENCE_MODEL_HPP
#define ONNX_INFERENCE_MODEL_HPP

#include <string>
#include <vector>
#include <memory>
#include <onnxruntime_cxx_api.h>

namespace onnx_inference {

class OnnxModel {
public:
    OnnxModel(const std::string& model_path);
    ~OnnxModel() = default;
    
    std::vector<float> run_inference(const std::vector<float>& input);
    
private:
    Ort::Env env_;
    Ort::SessionOptions session_options_;
    std::unique_ptr<Ort::Session> session_;
    
    std::vector<const char*> input_names_;
    std::vector<const char*> output_names_;
    std::vector<std::int64_t> input_shape_;
    std::vector<std::int64_t> output_shape_;
};

} // namespace onnx_inference

#endif // ONNX_INFERENCE_MODEL_HPP
