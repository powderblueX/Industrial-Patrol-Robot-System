#include <iostream>
#include <string>
#include "human_detector.h"

int main(int argc, char* argv[]) {
    // Create human detector instance
    HumanDetector detector;
    
    // Model path - can be specified via command line or use default path
    std::string modelPath = "../models/last_openvino_model/last.xml";
    
    if (argc > 1) {
        modelPath = argv[1];
    }
    
    // Confidence threshold - lowered to 0.1 to detect more potential humans
    float confidenceThreshold = 0.3;
    if (argc > 2) {
        confidenceThreshold = std::stof(argv[2]);
    }
    
    // Initialize detector
    if (!detector.initialize(modelPath, confidenceThreshold)) {
        std::cerr << "Failed to initialize detector!" << std::endl;
        return -1;
    }
    
    // Process video from camera
    detector.processVideo(0);
    
    return 0;
}