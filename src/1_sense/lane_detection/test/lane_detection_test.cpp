#include <gtest/gtest.h>
#include <filesystem>
#include <opencv2/opencv.hpp>
#include "lane_detection.h"
#include "lane_detection_config.h"
#include "test_helpers.h"


class LaneDetectionTest : public ::testing::Test {
protected:
    LaneDetectionParams default_params;
    std::string cwd = std::filesystem::current_path().parent_path().parent_path().string();
    std::string default_config_path = cwd + "/src/1_sense/lane_detection/config/default_config.yaml";
    std::string image_path = cwd + "/src/1_sense/lane_detection/test/images/";
    LaneDetection lane_detection{default_params};
    cv::Mat image;
    std::vector<std::string> image_files;

    void SetUp() override {
        // Load images from the specified directory
        load_config_from_yaml(default_config_path, default_params);
        cv::glob(image_path, image_files, false);
    }
};

TEST_F(LaneDetectionTest, TestLaneDetectionConfig) {
    // Example test case that checks if the config was loaded correctly
    EXPECT_EQ(default_params.nwindows, 27);  // Replace with actual values from your YAML config
    EXPECT_EQ(default_params.show_opencv_images, false);  // Replace with actual expected value
}

TEST_F(LaneDetectionTest, BasicProcessImageTest) {
    ASSERT_FALSE(image_files.empty()) << "No images found.";

    for (const auto& file : image_files) {
        std::cout << "Processing: " << file << std::endl;
        image = cv::imread(file, cv::IMREAD_COLOR);
        ASSERT_FALSE(image.empty()) << "Failed to load image: " << file;

        auto [left_fit, right_fit, mid_fit] = lane_detection.process_image(image, CurrentLane::RIGHT);

        ASSERT_FALSE(mid_fit.empty()) << "Middle lane fit is empty for image: " << file;
    }
}