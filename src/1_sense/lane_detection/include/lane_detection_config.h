#ifndef TOKYODRIFT_LANE_DETECTION_CONFIG_H
#define TOKYODRIFT_LANE_DETECTION_CONFIG_H

#include <stdint.h>
#include <iostream>
#include <yaml-cpp/yaml.h>
#include <opencv2/opencv.hpp>
#include <filesystem>
#include <omp.h>
#include <bits/stdc++.h>

#include "display_check.hpp"
#include "lane_detection_helper.hpp"

namespace fs = std::filesystem;

typedef struct {
    // sliding windows settings
    bool show_opencv_images = false;
    bool pub_images = false;
    bool interpolate = false;
    uint8_t nwindows = 0;
    uint8_t min_pixels = 0;
    float x_shift_factor = 0.0f;
    float x_shift_next_factor = 0.0f;
    float window_width_factor = 0.0f;
    float lane_distance_factor = 0.0f;
    float safe_lane_distance_factor = 0.0f;
    float left_init_factor = 0.0f;
    float right_init_factor = 0.0f;
    float valid_curve_factor = 0.0f;
    float next_init_factor = 0.0f;
    float max_init_offset_factor = 0.0f;
    float valid_band = 0.0f;
    uint8_t first_iters_each_side = 0;
    uint8_t iter_each_side = 0;

    uint16_t binary_adaptive_box = 0;
    int16_t binary_adaptive_subtract = 0;
    uint8_t num_threads = 0;
    std::string camcal_yaml = "";

    // POST INIT
    uint16_t bird_view_width = 0;
    uint16_t bird_view_height = 0;
    float px2m = 0.0f;
    float x0_m = 0.0f;
    uint16_t roi_height = 0;
    cv::Mat transformation_matrix = (cv::Mat_<double>(3, 3) << 
        0.0, 0.0, 0.0, 
        0.0, 0.0, 0.0, 
        0.0, 0.0, 0.0
      );
    uint16_t left_init_x = 0;
    uint16_t right_init_x = 0;
    uint16_t window_width = 0;
    uint16_t window_height = 0;
    uint16_t x_shift = 0;
    uint16_t x_shift_next = 0;
    uint16_t lane_distance = 0;
    uint16_t safe_lane_distance = 0;

    std::vector<int16_t> left_iters = {0};
    std::vector<int16_t> right_iters = {0};
    std::vector<int16_t> notfound_left_iters = {0};
    std::vector<int16_t> notfound_right_iters = {0};
} LaneDetectionParams;


void post_init_lane_detection_params(LaneDetectionParams* config);
cv::Mat_<double> parse_transformation_matrix(const std::string& M_string);
std::vector<cv::Point> parse_roi(const std::string& str);
uint16_t get_max_roi_height(std::vector<cv::Point> points);
void update_iters(LaneDetectionParams* config);
std::vector<int16_t> generate_iters(int16_t iters_each_side, CurrentLine current_lane);
std::ostream& operator<<(std::ostream& os, const LaneDetectionParams& config);

#endif // TOKYODRIFT_LANE_DETECTION_CONFIG_H