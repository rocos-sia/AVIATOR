#include "t6.hpp"

#include <onnxruntime_cxx_api.h>

#include <cmath>
#include <stdexcept>

namespace aviator::t6 {

struct Actor::Impl {
    Ort::Env env{ORT_LOGGING_LEVEL_WARNING, "aviator_t6"};
    Ort::SessionOptions options;
    std::unique_ptr<Ort::Session> session;
    std::string input_name, output_name;

    explicit Impl(const std::string& path) {
        options.SetIntraOpNumThreads(1);
        options.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
        session = std::make_unique<Ort::Session>(env, path.c_str(), options);
        if (session->GetInputCount() != 1 || session->GetOutputCount() != 1)
            throw std::runtime_error("t6 ONNX model must have one input and one output");
        const auto input_type = session->GetInputTypeInfo(0);
        const auto output_type = session->GetOutputTypeInfo(0);
        const auto input = input_type.GetTensorTypeAndShapeInfo();
        const auto output = output_type.GetTensorTypeAndShapeInfo();
        if (input.GetElementType() != ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT ||
            output.GetElementType() != ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT ||
            input.GetShape() != std::vector<int64_t>({1, 40}) ||
            output.GetShape() != std::vector<int64_t>({1, 2}))
            throw std::runtime_error("t6 ONNX interface must be float32 [1,40] -> [1,2]");
        Ort::AllocatorWithDefaultOptions allocator;
        input_name = session->GetInputNameAllocated(0, allocator).get();
        output_name = session->GetOutputNameAllocated(0, allocator).get();
    }
};

Actor::Actor(const std::string& onnx_path) : impl_(std::make_unique<Impl>(onnx_path)) {}
Actor::~Actor() = default;

Vec2 Actor::predict(const std::array<float, 40>& observation) const {
    for (float v : observation)
        if (!std::isfinite(v)) throw std::invalid_argument("Non-finite t6 observation");
    auto input_data = observation;
    constexpr std::array<int64_t, 2> shape{1, 40};
    auto memory = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
    auto input = Ort::Value::CreateTensor<float>(memory, input_data.data(), input_data.size(),
                                                  shape.data(), shape.size());
    const char* input_names[] = {impl_->input_name.c_str()};
    const char* output_names[] = {impl_->output_name.c_str()};
    auto outputs = impl_->session->Run(Ort::RunOptions{nullptr}, input_names, &input, 1,
                                       output_names, 1);
    if (outputs.size() != 1 || !outputs[0].IsTensor() ||
        outputs[0].GetTensorTypeAndShapeInfo().GetElementCount() != 2)
        throw std::runtime_error("Invalid t6 ONNX output");
    const float* action = outputs[0].GetTensorData<float>();
    if (!std::isfinite(action[0]) || !std::isfinite(action[1]) ||
        std::abs(action[0]) > 1 || std::abs(action[1]) > 1)
        throw std::runtime_error("Invalid t6 ONNX action");
    return {action[0], action[1]};
}

} // namespace aviator::t6
