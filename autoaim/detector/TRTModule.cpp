//
// Created by xinyang on 2021/4/8.
//

#include "TRTModule.hpp"
#include <fstream>
#include <filesystem>
#include <logger.h>
#include <cuda.h>
#include <cuda_runtime_api.h>
#include <NvInfer.h>
#include <NvOnnxParser.h>
#include <fmt/format.h>
#include <fmt/color.h>
#include <opencv2/imgproc.hpp>

#define TRT_ASSERT(expr)                                                      \
    do{                                                                       \
        if(!(expr)) {                                                         \
            fmt::print(fmt::fg(fmt::color::red), "assert fail: '" #expr "'"); \
            exit(-1);                                                         \
        }                                                                     \
    } while(0)

using namespace nvinfer1;
using namespace sample;

static inline size_t get_dims_size(const Dims &dims) {
    size_t sz = 1;
    for (int i = 0; i < dims.nbDims; i++) sz *= dims.d[i];
    return sz;
}

template<class F, class T, class ...Ts>
T reduce(F &&func, T x, Ts... xs) {
    if constexpr (sizeof...(Ts) > 0){
        return func(x, reduce(std::forward<F>(func), xs...));
    } else {
        return x;
    }
}

template<class T, class ...Ts>
T reduce_max(T x, Ts... xs) {
    return reduce([](auto &&a, auto &&b){return std::max(a, b);}, x, xs...);
}

template<class T, class ...Ts>
T reduce_min(T x, Ts... xs) {
    return reduce([](auto &&a, auto &&b){return std::min(a, b);}, x, xs...);
}

static inline bool is_overlap(const float pts1[8], const float pts2[8]) {
    cv::Rect2f bbox1, bbox2;
    bbox1.x = reduce_min(pts1[0], pts1[2], pts1[4], pts1[6]);
    bbox1.y = reduce_min(pts1[1], pts1[3], pts1[5], pts1[7]);
    bbox1.width = reduce_max(pts1[0], pts1[2], pts1[4], pts1[6]) - bbox1.x;
    bbox1.height = reduce_max(pts1[1], pts1[3], pts1[5], pts1[7]) - bbox1.y;
    bbox2.x = reduce_min(pts2[0], pts2[2], pts2[4], pts2[6]);
    bbox2.y = reduce_min(pts2[1], pts2[3], pts2[5], pts2[7]);
    bbox2.width = reduce_max(pts2[0], pts2[2], pts2[4], pts2[6]) - bbox2.x;
    bbox2.height = reduce_max(pts2[1], pts2[3], pts2[5], pts2[7]) - bbox2.y;
    return (bbox1 & bbox2).area() > 0;
}

static inline int argmax(const float *ptr, int len) {
    int max_arg = 0;
    for (int i = 1; i < len; i++) {
        if (ptr[i] > ptr[max_arg]) max_arg = i;
    }
    return max_arg;
}

constexpr float inv_sigmoid(float x) {
    return -std::log(1 / x - 1);
}

constexpr float sigmoid(float x) {
    return 1 / (1 + std::exp(-x));
}

TRTModule::TRTModule(const std::string &onnx_file) {
    std::filesystem::path onnx_file_path(onnx_file);
    auto cache_file_path = onnx_file_path;
    cache_file_path.replace_extension("cache");
    if (std::filesystem::exists(cache_file_path)) {
        build_engine_from_cache(cache_file_path.c_str());
    } else {
        build_engine_from_onnx(onnx_file_path.c_str());
        cache_engine(cache_file_path.c_str());
    }
    TRT_ASSERT((context = engine->createExecutionContext()) != nullptr);
    TRT_ASSERT((input_idx = engine->getBindingIndex("input")) == 0);
    TRT_ASSERT((output_idx = engine->getBindingIndex("output-topk")) == 1);
    auto input_dims = engine->getBindingDimensions(input_idx);
    auto output_dims = engine->getBindingDimensions(output_idx);
    input_sz = get_dims_size(input_dims);
    output_sz = get_dims_size(output_dims);
    TRT_ASSERT(cudaMalloc(&device_buffer[input_idx], input_sz * sizeof(float)) == 0);
    TRT_ASSERT(cudaMalloc(&device_buffer[output_idx], output_sz * sizeof(float)) == 0);
    TRT_ASSERT(cudaStreamCreate(&stream) == 0);
    output_buffer = new float[output_sz];
    TRT_ASSERT(output_buffer != nullptr);
}

TRTModule::~TRTModule() {
    delete[] output_buffer;
    cudaStreamDestroy(stream);
    cudaFree(device_buffer[output_idx]);
    cudaFree(device_buffer[input_idx]);
    engine->destroy();
}

void TRTModule::build_engine_from_onnx(const std::string &onnx_file) {
    std::cout << "[INFO]: build engine from onnx" << std::endl;
    auto builder = createInferBuilder(gLogger);
    TRT_ASSERT(builder != nullptr);
    const auto explicitBatch = 1U << static_cast<uint32_t>(NetworkDefinitionCreationFlag::kEXPLICIT_BATCH);
    auto network = builder->createNetworkV2(explicitBatch);
    TRT_ASSERT(network != nullptr);
    auto parser = nvonnxparser::createParser(*network, gLogger);
    TRT_ASSERT(parser != nullptr);
    parser->parseFromFile(onnx_file.c_str(), static_cast<int>(ILogger::Severity::kINFO));
    auto yolov5_output = network->getOutput(0);
    auto slice_layer = network->addSlice(*yolov5_output, Dims3{0, 0, 8}, Dims3{1, 15120, 1}, Dims3{1, 1, 1});
    auto yolov5_conf = slice_layer->getOutput(0);
    auto shuffle_layer = network->addShuffle(*yolov5_conf);
    shuffle_layer->setReshapeDimensions(Dims2{1, 15120});
    yolov5_conf = shuffle_layer->getOutput(0);
    auto topk_layer = network->addTopK(*yolov5_conf, TopKOperation::kMAX, TOPK_NUM, 1 << 1);
    auto topk_idx = topk_layer->getOutput(1);
    auto gather_layer = network->addGather(*yolov5_output, *topk_idx, 1);
    gather_layer->setNbElementWiseDims(1);
    auto yolov5_output_topk = gather_layer->getOutput(0);
    yolov5_output_topk->setName("output-topk");
    network->getInput(0)->setName("input");
    network->markOutput(*yolov5_output_topk);
    network->unmarkOutput(*yolov5_output);
    auto config = builder->createBuilderConfig();
    if (builder->platformHasFastFp16()) {
        std::cout << "[INFO]: platform support fp16, enable fp16" << std::endl;
        config->setFlag(BuilderFlag::kFP16);
    } else {
        std::cout << "[INFO]: platform do not support fp16, enable fp32" << std::endl;
    }
    size_t free, total;
    cuMemGetInfo(&free, &total);
    std::cout << "[INFO]: total gpu mem: " << (total >> 20) << "MB, free gpu mem: " << (free >> 20) << "MB" << std::endl;
    std::cout << "[INFO]: max workspace size will use all of free gpu mem" << std::endl;
    config->setMaxWorkspaceSize(free);
    TRT_ASSERT((engine = builder->buildEngineWithConfig(*network, *config)) != nullptr);
    config->destroy();
    parser->destroy();
    network->destroy();
    builder->destroy();
}

void TRTModule::build_engine_from_cache(const std::string &cache_file) {
    std::cout << "[INFO]: build engine from cache" << std::endl;
    std::ifstream ifs(cache_file, std::ios::binary);
    ifs.seekg(0, std::ios::end);
    size_t sz = ifs.tellg();
    ifs.seekg(0, std::ios::beg);
    auto buffer = std::make_unique<char[]>(sz);
    ifs.read(buffer.get(), sz);
    auto runtime = createInferRuntime(gLogger);
    TRT_ASSERT(runtime != nullptr);
    TRT_ASSERT((engine = runtime->deserializeCudaEngine(buffer.get(), sz)) != nullptr);
    runtime->destroy();
}

void TRTModule::cache_engine(const std::string &cache_file) {
    auto engine_buffer = engine->serialize();
    TRT_ASSERT(engine_buffer != nullptr);
    std::ofstream ofs(cache_file, std::ios::binary);
    ofs.write(static_cast<const char *>(engine_buffer->data()), engine_buffer->size());
    engine_buffer->destroy();
}

std::vector<bbox_t> TRTModule::operator()(const cv::Mat &src) const {
    // pre-process [bgr2rgb & resize]
    cv::Mat x;
    float fx = (float) src.cols / 640.f, fy = (float) src.rows / 384.f;
    cv::cvtColor(src, x, cv::COLOR_BGR2RGB);
    if (src.cols != 640 || src.rows != 384) {
        cv::resize(x, x, {640, 384});
    }
    x.convertTo(x, CV_32F);

    // run model
    cudaMemcpyAsync(device_buffer[input_idx], x.data, input_sz * sizeof(float), cudaMemcpyHostToDevice, stream);
    context->enqueue(1, device_buffer, stream, nullptr);
    cudaMemcpyAsync(output_buffer, device_buffer[output_idx], output_sz * sizeof(float), cudaMemcpyDeviceToHost,
                    stream);
    cudaStreamSynchronize(stream);

    // post-process [nms]
    std::vector<bbox_t> rst;
    rst.reserve(TOPK_NUM);
    std::vector<uint8_t> removed(TOPK_NUM);
    for (int i = 0; i < TOPK_NUM; i++) {
        auto *box_buffer = output_buffer + i * 20;  // 20->23
        if (box_buffer[8] < inv_sigmoid(KEEP_THRES)) break;
        if (removed[i]) continue;
        rst.emplace_back();
        auto &box = rst.back();
        memcpy(&box.pts, box_buffer, 8 * sizeof(float));
        for (auto &pt : box.pts) pt.x *= fx, pt.y *= fy;
        box.confidence = sigmoid(box_buffer[8]);
        box.color_id = argmax(box_buffer + 9, 4);
        box.tag_id = argmax(box_buffer + 13, 7);
        for (int j = i + 1; j < TOPK_NUM; j++) {
            auto *box2_buffer = output_buffer + j * 20;
            if (box2_buffer[8] < inv_sigmoid(KEEP_THRES)) break;
            if (removed[j]) continue;
            if (is_overlap(box_buffer, box2_buffer)) removed[j] = true;
        }
    }

    return rst;
}
