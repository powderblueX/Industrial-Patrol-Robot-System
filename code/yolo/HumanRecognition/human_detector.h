#pragma once

#include <string>
#include <vector>
#include <queue>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <opencv2/opencv.hpp>
#include <openvino/openvino.hpp>

// 定义检测结果结构体，包含边界框和置信度
struct Detection {
    cv::Rect box;
    float confidence;
    
    Detection(cv::Rect r, float conf) : box(r), confidence(conf) {}
};

// 定义帧处理任务结构体
struct FrameTask {
    cv::Mat frame;
    int64_t timestamp;
};

class HumanDetector {
public:
    // Constructor
    HumanDetector();
    ~HumanDetector();
    
    // Initialize detector
    bool initialize(const std::string& modelPath, float confidenceThreshold = 0.5);
    
    // Process video from camera
    bool processVideo(int cameraId = 0);
    
    // Process single frame
    cv::Mat processFrame(const cv::Mat& frame);

private:
    // OpenVINO inference engine
    ov::Core core;
    ov::CompiledModel compiledModel;
    ov::InferRequest inferRequest;
    
    // Model parameters
    float confidenceThreshold;
    int inputWidth;
    int inputHeight;
    
    // Thread pool related
    std::vector<std::thread> workerThreads;
    std::queue<FrameTask> frameQueue;
    std::mutex queueMutex;
    std::condition_variable queueCondition;
    std::atomic<bool> stopProcessing;
    int numThreads;
    
    // Worker thread function
    void workerThread();
    
    // Postprocessing method
    std::vector<Detection> postprocess(const cv::Mat& frame, const ov::Tensor& outputTensor);
};