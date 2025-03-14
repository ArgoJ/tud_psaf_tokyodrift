#ifndef TOKYODRIFT_LANE_DETECTION_TEST_HELPERS_H
#define TOKYODRIFT_LANE_DETECTION_TEST_HELPERS_H

#include <iostream>
#include <yaml-cpp/yaml.h>
#include <opencv2/opencv.hpp>
#include "lane_detection.h" 
#include "lane_detection_config.h"
#include "lane_detection_helper.hpp"

void print_all_keys(const YAML::Node& node, const std::string& prefix = "");
void load_config_from_yaml(const std::string& filepath, LaneDetectionParams& config);
LaneDetectionParams default_config();
LaneDetectionParams debug_config();
LaneDetectionParams ros_debug_config();

#endif // TOKYODRIFT_LANE_DETECTION_TEST_HELPERS_H
