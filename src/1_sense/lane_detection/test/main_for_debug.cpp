#include <iostream>
#include <yaml-cpp/yaml.h>
#include <opencv2/opencv.hpp>
#include "lane_detection.h" 
#include "lane_detection_config.h"
#include "lane_detection_helper.h"
#include "test_helpers.h"


int main(int argc, char** argv) {
    // Check for the configuration file
    if (argc < 3) {
        std::cerr << "Usage: " << argv[0] << " <path_to_config.yaml> <path_to_image_or_directory>" << std::endl;
        return -1;
    }

    std::string config_path = argv[1];
    std::string image_path = argv[2];

    // Load configuration
    LaneDetectionParams config;
    try {
        load_config_from_yaml(config_path, config);
        std::cout << "Configuration loaded successfully!" << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "Failed to load configuration: " << e.what() << std::endl;
        return 1;
    }

    std::cout << config << std::endl;

    // Initialize LaneDetection
    LaneDetection lane_detection(config);  // Enable visualization if needed

    // Process image(s)
    cv::Mat image;
    std::vector<std::string> image_files;

    cv::glob(image_path, image_files, false);
    if (image_files.empty()) {
        std::cerr << "No images found at " << image_path << std::endl;
        return -1;
    }

    for (const auto& file : image_files) {
        std::cout << "Processing: " << file << std::endl;
        image = cv::imread(file, cv::IMREAD_COLOR);
        if (image.empty()) {
            std::cerr << "Failed to load image: " << file << std::endl;
            continue;
        }

        // Process the image
        auto [left_fit, right_fit, mid_fit] = lane_detection.process_image(image);
    }

    return 0;
}
