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

// Define detection result structure, containing bounding box and confidence
struct Detection {
    cv::Rect box;
    float confidence;
    int classId;

    Detection(const cv::Rect& r, float conf, int cid) : box(r), confidence(conf), classId(cid) {}
};

// Define frame processing task structure
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