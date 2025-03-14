#include <gtest/gtest.h>
#include <filesystem>
#include <opencv2/opencv.hpp>
#include "start_detector.h"
#include "start_detector_config.h"


class StartDetectorTest : public ::testing::Test {
protected:
    std::string cwd = std::filesystem::current_path().parent_path().parent_path().string();
    StartDetectorParams default_params;
};


//Test for pictures with stopsigns 
TEST_F(StartDetectorTest, PicturesWithStopsign_Test) {
    StartDetector start_detector;
    std::vector<std::string> image_files;

    

    std::string image_path = cwd + "/src/1_sense/start_box/test/images/sign/";
    std::cout << "image path: " << image_path << std::endl;
    // Load images from the specified directory
    cv::glob(image_path, image_files, false);
    std::cout << " image files[0]: " << image_files.at(0) << std::endl; 

    ASSERT_FALSE(image_files.empty()) << "No images found.";

    for (const auto& file : image_files) {
        std::cout << "Processing: " << file << std::endl;
        cv::Mat image = cv::imread(file, cv::IMREAD_COLOR);
        //ASSERT_FALSE(image.empty()) << "Failed to load image: " << file;

        auto binary_image = start_detector.convert_image(image);
        auto is_visible = start_detector.is_stop_sign_still_visible(binary_image, default_params.window_size, default_params.pixels_threshold, default_params.squares_threshold, default_params.step_size);
        std::cout << "conversion successful. " << std::endl;
        EXPECT_EQ(is_visible, true) << "Stop sign not visible - although there is one for image: " << file;
    }
}
//Test for pictures without stopsigns 
TEST_F(StartDetectorTest, PicturesWithoutStopSign_Test) {
    StartDetector start_detector;
    std::vector<std::string> image_files;

    std::string cwd = std::filesystem::current_path().string();
    std::string image_path = cwd + "/src/1_sense/start_box/test/images/no_sign"; //TODO change path
    // Load images from the specified directory
    cv::glob(image_path, image_files, false);

    ASSERT_FALSE(!image_files.empty()) << "No images found.";

    for (const auto& file : image_files) {
        std::cout << "Processing: " << file << std::endl;
        cv::Mat image = cv::imread(file, cv::IMREAD_COLOR);
        ASSERT_FALSE(image.empty()) << "Failed to load image: " << file;

        auto binary_image = start_detector.convert_image(image);
        auto is_visible = start_detector.is_stop_sign_still_visible(binary_image, default_params.window_size, default_params.pixels_threshold, default_params.squares_threshold, default_params.step_size);

        ASSERT_FALSE(is_visible) << "Stop sign visible - although there is none for image: " << file;
    }
}