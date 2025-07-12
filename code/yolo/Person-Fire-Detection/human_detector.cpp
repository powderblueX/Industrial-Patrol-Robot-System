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
        if (modelExtension == "xml" || modelExtension == "onnx") {
            // Load OpenVINO IR model
            compiledModel = core.compile_model(modelPath);
        }
        else if (modelExtension == "pt") {
            // Load PyTorch model using ONNX as intermediate format
            std::cout << "PyTorch models need to be converted to ONNX or OpenVINO format first" << std::endl;
            return false;
        }
        else {
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
            inputShape = { 1, 3, 640, 640 };
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
    } else {
        std::cout << "Camera opened successfully." << std::endl;
    }

    cv::Mat frame;
    // Variables for FPS calculation
    int frameCount = 0;
    double startTime = (double)cv::getTickCount();
    double fps = 0;

    while (true) {
        // Read frame
        cap >> frame;
        if (frame.empty()) {
            std::cerr << "Video stream ended or cannot grab frame." << std::endl;
            break;
        } else {
            std::cout << "[Frame] Grabbed successfully." << std::endl;
        }

        // 单线程：直接处理和显示
        cv::Mat resultFrame = processFrame(frame);
        cv::imshow("HumanDetection", resultFrame);
        int key = cv::waitKey(1);
        if (key == 'q' || key == 27) { // 支持q或ESC退出
            break;
        }

        // Calculate FPS
        frameCount++;
        double currentTime = (double)cv::getTickCount();
        double elapsedTime = (currentTime - startTime) / cv::getTickFrequency();
        if (elapsedTime >= 1.0) {
            fps = frameCount / elapsedTime;
            frameCount = 0;
            startTime = currentTime;
            std::cout << "FPS: " << fps << std::endl;
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
            // Process frame
            cv::Mat resultFrame = processFrame(task.frame);

            // Show result
            cv::imshow("HumanDetection", resultFrame);
        }
    }
}

cv::Mat HumanDetector::processFrame(const cv::Mat& frame) {
    cv::Mat resultFrame = frame.clone();
    // 显示原始摄像头画面，便于调试灰屏
    // cv::imshow("DebugRaw", frame);

    try {
        // Preprocess image
        cv::Mat blobImage;
        cv::resize(frame, blobImage, cv::Size(inputWidth, inputHeight));
        // Convert to float and normalize
        blobImage.convertTo(blobImage, CV_32F, 1.0 / 255.0);

        // YOLOv8 model input format is NCHW
        std::vector<cv::Mat> channels;
        cv::split(blobImage, channels);

        // Create input tensor
        ov::Tensor inputTensor = ov::Tensor(ov::element::f32, { 1, 3, static_cast<size_t>(inputHeight), static_cast<size_t>(inputWidth) });
        float* inputData = inputTensor.data<float>();

        // Fill data
        int channelSize = inputHeight * inputWidth;
        for (int c = 0; c < 3; c++) {
            cv::Mat channel = channels[c];
            memcpy(inputData + c * channelSize, channel.data, channelSize * sizeof(float));
        }

        // 创建本地推理请求，避免多线程冲突
        auto inferRequest = compiledModel.create_infer_request();
        inferRequest.set_input_tensor(0, inputTensor);
        inferRequest.infer();
        ov::Tensor outputTensor = inferRequest.get_output_tensor(0);

        // Postprocess to get detections
        std::vector<Detection> detections = postprocess(frame, outputTensor);

        // Draw detection results
        static const char* classNames[] = { "person", "fire", "smoke" };
        static const cv::Scalar classColors[] = { cv::Scalar(0,255,0), cv::Scalar(0,0,255), cv::Scalar(255,165,0) };
        for (const auto& detection : detections) {
            float confidence = std::min(std::max(detection.confidence, 0.0f), 1.0f);
            int cid = detection.classId;
            std::string label;
            cv::Scalar color(0, 255, 0);
            if (cid >= 0 && cid < 3) {
                label = std::string(classNames[cid]) + ": " + std::to_string(int(confidence * 100)) + "%";
                color = classColors[cid];
            }
            else {
                label = "unknown: " + std::to_string(int(confidence * 100)) + "%";
            }
            cv::rectangle(resultFrame, detection.box, color, 2);
            int baseLine;
            cv::Size labelSize = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseLine);
            cv::rectangle(resultFrame,
                cv::Point(detection.box.x, detection.box.y - labelSize.height - baseLine - 10),
                cv::Point(detection.box.x + labelSize.width, detection.box.y),
                color, cv::FILLED);
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
        ov::Shape outputShape = outputTensor.get_shape();
        std::cout << "Output shape: [";
        for (size_t i = 0; i < outputShape.size(); i++) {
            std::cout << outputShape[i] << (i < outputShape.size() - 1 ? ", " : "");
        }
        std::cout << "]" << std::endl;

        // Only process [1,6,8400] format (YOLOv8 multi-label binary classification)
        if (outputShape.size() == 3 && outputShape[1] == 6) {
            int num_boxes = outputShape[2];
            int num_classes = 2;
            int input_w = inputWidth;
            int input_h = inputHeight;
            int orig_w = frame.cols;
            int orig_h = frame.rows;

            // letterbox params
            float ratio = std::min(input_w / (float)orig_w, input_h / (float)orig_h);
            int new_w = int(orig_w * ratio);
            int new_h = int(orig_h * ratio);
            int dw = (input_w - new_w) / 2;
            int dh = (input_h - new_h) / 2;

            float class_thresh[2] = { 0.6f, 0.55f }; // threshold

            // Iterate each anchor
            for (int i = 0; i < num_boxes; ++i) {
                float cx = output[0 * num_boxes + i];
                float cy = output[1 * num_boxes + i];
                float w = output[2 * num_boxes + i];
                float h = output[3 * num_boxes + i];
                // Restore to letterbox input size
                float x1 = cx - w / 2;
                float y1 = cy - h / 2;
                float x2 = cx + w / 2;
                float y2 = cy + h / 2;
                // Map back to original image
                x1 = (x1 - dw) / ratio;
                y1 = (y1 - dh) / ratio;
                x2 = (x2 - dw) / ratio;
                y2 = (y2 - dh) / ratio;
                int x = int(x1);
                int y = int(y1);
                int width = int(x2 - x1);
                int height = int(y2 - y1);
                cv::Rect box(x, y, width, height);
                for (int cid = 0; cid < num_classes; ++cid) {
                    float logit = output[(4 + cid) * num_boxes + i];
                    float conf = 1.0f / (1.0f + std::exp(-logit));
                    if (conf > class_thresh[cid] && std::abs(conf - 0.5f) > 1e-6) {
                        detections.push_back(Detection(box, conf, cid));
                    }
                }
            }
        }
        else {
            // Other formats fallback to old logic
            std::cout << "Format not matching [1,6,8400], fallback to old logic." << std::endl;
            // ...original fallback code can be placed here...
        }

        // NMS
        std::vector<int> indices;
        std::vector<cv::Rect> boxes;
        std::vector<float> scores;
        for (const auto& det : detections) {
            boxes.push_back(det.box);
            scores.push_back(det.confidence);
        }
        cv::dnn::NMSBoxes(boxes, scores, 0.001f, 0.45f, indices);
        std::vector<Detection> results;
        for (int idx : indices) {
            const Detection& det = detections[idx];
            std::cout << "Class: " << det.classId << ", Confidence: " << det.confidence
                      << ", Box: [" << det.box.x << "," << det.box.y << "," << det.box.width << "," << det.box.height << "]" << std::endl;
            results.push_back(det);
        }
        std::cout << "Detected " << results.size() << " objects after NMS" << std::endl;
        return results;
    }
    catch (const std::exception& e) {
        std::cerr << "Error in postprocessing: " << e.what() << std::endl;
        return detections;
    }
}
