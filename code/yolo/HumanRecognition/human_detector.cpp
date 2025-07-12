#define _CRT_SECURE_NO_DEPRECATE
#include "human_detector.h"
#include <iostream>
#include <algorithm>

HumanDetector::HumanDetector() : confidenceThreshold(0.5), inputWidth(640), inputHeight(640), 
                                stopProcessing(false), numThreads(4) {
}

HumanDetector::~HumanDetector() {
    stopProcessing = true;
    queueCondition.notify_all();
    for (auto& thread : workerThreads) {
        if (thread.joinable()) {
            thread.join();
        }
    }
}

bool HumanDetector::initialize(const std::string& modelPath, float confThreshold) {
    try {
        // Set confidence threshold
        confidenceThreshold = confThreshold;
        
        // Load model
        std::cout << "Loading model: " << modelPath << std::endl;
        
        // Determine model type and load
        std::string modelExtension = modelPath.substr(modelPath.find_last_of(".") + 1);
        if (modelExtension == "xml") {
            // Load OpenVINO IR model
            compiledModel = core.compile_model(modelPath);
        } else if (modelExtension == "onnx") {
            // Load ONNX model
            compiledModel = core.compile_model(modelPath);
        } else if (modelExtension == "pt") {
            // Load PyTorch model using ONNX as intermediate format
            std::cout << "PyTorch models need to be converted to ONNX or OpenVINO format first" << std::endl;
            return false;
        } else {
            std::cout << "Unsupported model format: " << modelExtension << std::endl;
            return false;
        }
        
        // Create inference request
        inferRequest = compiledModel.create_infer_request();
        
        // Get input shape from the model
        ov::Shape inputShape;
        try {
            ov::Output<const ov::Node> input = compiledModel.input();
            inputShape = input.get_shape();
        }
        catch (const std::exception& e) {
            std::cout << "Warning: Could not get input name: " << e.what() << std::endl;
            std::cout << "Using default input parameters." << std::endl;
            // Use default shape if can't get from model
            inputShape = {1, 3, 640, 640};
        }
        
        // Get input dimensions
        if (inputShape.size() == 4) {
            inputHeight = inputShape[2];
            inputWidth = inputShape[3];
        }
        
        std::cout << "Model loaded successfully. Input size: " << inputWidth << "x" << inputHeight << std::endl;
        return true;
    }
    catch (const std::exception& e) {
        std::cerr << "Initialization failed: " << e.what() << std::endl;
        return false;
    }
}

bool HumanDetector::processVideo(int cameraId) {
    // Open camera
    cv::VideoCapture cap(cameraId);
    if (!cap.isOpened()) {
        std::cerr << "Cannot open camera" << std::endl;
        return false;
    }
    
    // 启动工作线程
    for (int i = 0; i < numThreads; ++i) {
        workerThreads.emplace_back(&HumanDetector::workerThread, this);
    }
    
    cv::Mat frame;
    // 用于计算FPS的变量
    int frameCount = 0;
    double startTime = cv::getTickCount();
    double fps = 0;
    
    while (true) {
        // Read frame
        cap >> frame;
        if (frame.empty()) {
            std::cerr << "Video stream ended" << std::endl;
            break;
        }
        
        // 创建帧任务
        FrameTask task;
        task.frame = frame.clone();
        task.timestamp = cv::getTickCount();
        
        // 将任务添加到队列
        {
            std::lock_guard<std::mutex> lock(queueMutex);
            frameQueue.push(task);
        }
        queueCondition.notify_one();
        
        // 计算FPS
        frameCount++;
        double currentTime = cv::getTickCount();
        double elapsedTime = (currentTime - startTime) / cv::getTickFrequency();
        if (elapsedTime >= 1.0) {
            fps = frameCount / elapsedTime;
            frameCount = 0;
            startTime = currentTime;
            std::cout << "FPS: " << fps << std::endl;
        }
        
        // Exit on ESC key
        if (cv::waitKey(1) == 'q') {
            break;
        }
    }
    
    // 停止处理
    stopProcessing = true;
    queueCondition.notify_all();
    
    // 等待所有线程完成
    for (auto& thread : workerThreads) {
        if (thread.joinable()) {
            thread.join();
        }
    }
    
    cap.release();
    cv::destroyAllWindows();
    return true;
}

void HumanDetector::workerThread() {
    while (!stopProcessing) {
        FrameTask task;
        {
            std::unique_lock<std::mutex> lock(queueMutex);
            queueCondition.wait(lock, [this] { 
                return !frameQueue.empty() || stopProcessing; 
            });
            
            if (stopProcessing && frameQueue.empty()) {
                break;
            }
            
            if (!frameQueue.empty()) {
                task = frameQueue.front();
                frameQueue.pop();
            }
        }
        
        if (!task.frame.empty()) {
            // 处理帧
            cv::Mat resultFrame = processFrame(task.frame);
            
            // 显示结果
            cv::imshow("HumanDetection", resultFrame);
        }
    }
}

cv::Mat HumanDetector::processFrame(const cv::Mat& frame) {
    cv::Mat resultFrame = frame.clone();
    
    try {
        // Preprocess image
        cv::Mat blobImage;
        cv::resize(frame, blobImage, cv::Size(inputWidth, inputHeight));
        // Convert to float and normalize
        blobImage.convertTo(blobImage, CV_32F, 1.0/255.0);
        
        // YOLOv8 model input format is NCHW
        std::vector<cv::Mat> channels;
        cv::split(blobImage, channels);
        
        // Create input tensor
        ov::Tensor inputTensor = ov::Tensor(ov::element::f32, {1, 3, static_cast<size_t>(inputHeight), static_cast<size_t>(inputWidth)});
        float* inputData = inputTensor.data<float>();
        
        // Fill data
        int channelSize = inputHeight * inputWidth;
        for (int c = 0; c < 3; c++) {
            cv::Mat channel = channels[c];
            memcpy(inputData + c * channelSize, channel.data, channelSize * sizeof(float));
        }
        
        // Set input data - use index 0 since we don't have the name
        inferRequest.set_input_tensor(0, inputTensor);
        
        // Run inference
        inferRequest.infer();
        
        // Get output - use index 0 since we don't have the name
        ov::Tensor outputTensor = inferRequest.get_output_tensor(0);
        
        // Postprocess to get detections
        std::vector<Detection> detections = postprocess(frame, outputTensor);
        
        // Draw detection results
        for (const auto& detection : detections) {
            // Get confidence score and clamp it to 0-100%
            float confidence = std::min(std::max(detection.confidence, 0.0f), 1.0f);
            
            // Draw bounding box
            cv::rectangle(resultFrame, detection.box, cv::Scalar(0, 255, 0), 2);
            
            // Draw label with actual confidence (formatted to 2 decimal places)
            char confidenceStr[10];
            sprintf(confidenceStr, "%.2f", confidence * 100);
            std::string label = "person: " + std::string(confidenceStr) + "%";
            
            int baseLine;
            cv::Size labelSize = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseLine);
            cv::rectangle(resultFrame, 
                         cv::Point(detection.box.x, detection.box.y - labelSize.height - baseLine - 10),
                         cv::Point(detection.box.x + labelSize.width, detection.box.y),
                         cv::Scalar(0, 255, 0), cv::FILLED);
            cv::putText(resultFrame, label, cv::Point(detection.box.x, detection.box.y - baseLine - 5),
                       cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 1);
        }
    }
    catch (const std::exception& e) {
        std::cerr << "Error processing frame: " << e.what() << std::endl;
    }
    
    return resultFrame;
}

std::vector<Detection> HumanDetector::postprocess(const cv::Mat& frame, const ov::Tensor& outputTensor) {
    std::vector<Detection> detections;
    
    try {
        // Get output data
        const float* output = outputTensor.data<const float>();
        
        // Get output shape
        ov::Shape outputShape = outputTensor.get_shape();
        
        // Debug output shape
        std::cout << "Output shape: [";
        for (size_t i = 0; i < outputShape.size(); i++) {
            std::cout << outputShape[i] << (i < outputShape.size() - 1 ? ", " : "");
        }
        std::cout << "]" << std::endl;
        
        // Check if we have the [1, 5, 8400] output format for YOLOv8-seg model
        if (outputShape.size() == 3 && outputShape[1] == 5) {
            // For YOLOv8-seg models, the output is [1, 5, 8400] where:
            // - 5 = 4 (box coordinates) + 1 (confidence)
            // - 8400 = number of detection boxes
            
            int num_boxes = outputShape[2];
            float* boxes = new float[num_boxes * 4];  // x, y, w, h for each box
            float* scores = new float[num_boxes];     // confidence scores
            
            // Extract boxes and confidence scores
            for (int i = 0; i < num_boxes; ++i) {
                boxes[i*4 + 0] = output[0 * num_boxes + i]; // x
                boxes[i*4 + 1] = output[1 * num_boxes + i]; // y
                boxes[i*4 + 2] = output[2 * num_boxes + i]; // w
                boxes[i*4 + 3] = output[3 * num_boxes + i]; // h
                scores[i] = output[4 * num_boxes + i];      // confidence
            }
            
            // For each detection box
            for (int i = 0; i < num_boxes; ++i) {
                float confidence = scores[i];
                
                // Filter by confidence threshold
                if (confidence >= confidenceThreshold) {
                    // Extract box coordinates (already in xywh format, normalized)
                    float x = boxes[i*4 + 0];
                    float y = boxes[i*4 + 1];
                    float w = boxes[i*4 + 2];
                    float h = boxes[i*4 + 3];
                    
                    // Convert normalized coordinates to pixel coordinates
                    int img_width = frame.cols;
                    int img_height = frame.rows;
                    
                    // Convert center-based coordinates to top-left coordinates
                    int left = static_cast<int>((x - w/2) * img_width);
                    int top = static_cast<int>((y - h/2) * img_height);
                    int width = static_cast<int>(w * img_width);
                    int height = static_cast<int>(h * img_height);
                    
                    // Ensure box is within image boundaries
                    left = std::max(0, std::min(left, img_width - 1));
                    top = std::max(0, std::min(top, img_height - 1));
                    width = std::min(width, img_width - left);
                    height = std::min(height, img_height - top);
                    
                    // Add detection
                    Detection det(cv::Rect(left, top, width, height), confidence);
                    detections.push_back(det);
                }
            }
            
            // Clean up
            delete[] boxes;
            delete[] scores;
        }
        else {
            // Try the old postprocessing methods for other formats
            // Case 1: [1, 84, 8400] or [1, classes+5, boxes]
            if (outputShape.size() == 3 && outputShape[1] > outputShape[2]) {
                // ...existing code for other formats...
                std::cout << "Format [1, " << outputShape[1] << ", " << outputShape[2] 
                          << "] not matching our primary model format [1, 5, 8400]" << std::endl;
            }
            // Case 2: [1, 8400, 84] or [1, boxes, classes+5]
            else if (outputShape.size() == 3 && outputShape[1] < outputShape[2]) {
                // ...existing code for other formats...
                std::cout << "Format [1, " << outputShape[1] << ", " << outputShape[2] 
                          << "] not matching our primary model format [1, 5, 8400]" << std::endl;
            }
            // Other non-standard formats
            else {
                std::cout << "Warning: Unexpected output tensor shape. Using fallback approach." << std::endl;
                
                // ...existing fallback code...
            }
        }
        
        // Apply non-maximum suppression to eliminate overlapping boxes
        std::vector<Detection> nms_result;
        if (!detections.empty()) {
            // Sort detections by confidence (highest first)
            std::sort(detections.begin(), detections.end(), 
                     [](const Detection& a, const Detection& b) { return a.confidence > b.confidence; });
            
            // Apply NMS
            std::vector<bool> keep(detections.size(), true);
            for (size_t i = 0; i < detections.size(); i++) {
                if (!keep[i]) continue;
                
                const cv::Rect& boxA = detections[i].box;
                
                for (size_t j = i + 1; j < detections.size(); j++) {
                    if (!keep[j]) continue;
                    
                    const cv::Rect& boxB = detections[j].box;
                    
                    // Calculate intersection area
                    cv::Rect intersection = boxA & boxB;
                    float intersectionArea = intersection.width * intersection.height;
                    
                    // Calculate union area
                    float unionArea = (boxA.width * boxA.height) + (boxB.width * boxB.height) - intersectionArea;
                    
                    // Calculate IoU
                    float iou = intersectionArea / unionArea;
                    
                    // Suppress boxes with high IoU
                    if (iou > 0.45) {
                        keep[j] = false;
                    }
                }
            }
            
            // Keep only non-suppressed detections
            for (size_t i = 0; i < detections.size(); i++) {
                if (keep[i]) {
                    nms_result.push_back(detections[i]);
                }
            }
        }
        
        // Print number of detections found after NMS
        std::cout << "Detected " << nms_result.size() << " objects after NMS" << std::endl;
        
        return nms_result;
    }
    catch (const std::exception& e) {
        std::cerr << "Error in postprocessing: " << e.what() << std::endl;
        return detections;
    }
}
