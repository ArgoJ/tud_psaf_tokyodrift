#include "test_helpers.h"


void print_all_keys(const YAML::Node& node, const std::string& prefix) {
    if (node.IsMap()) {
        for (const auto& it : node) {
            std::string key = it.first.as<std::string>();  // Get the key
            std::cout << prefix << key << std::endl;  // Print the key
            
            // Recursively call print_all_keys to print keys of sub-nodes
            print_all_keys(it.second, prefix + key + ".");
        }
    } else if (node.IsSequence()) {
        // If it's a sequence, we can print the index
        for (std::size_t i = 0; i < node.size(); ++i) {
            print_all_keys(node[i], prefix + std::to_string(i) + ".");
        }
    }
}

void load_config_from_yaml(const std::string& filepath, LaneDetectionParams& config) {
    try {
        YAML::Node yaml_config = YAML::LoadFile(filepath);

        if (yaml_config["lane_detection"]["ros__parameters"]) {
            YAML::Node params = yaml_config["lane_detection"]["ros__parameters"];

            // std::cout << "YAML Params:" << std::endl;
            // print_all_keys(yaml_config);
            // print_all_keys(params);
            config.show_opencv_images = params["show_opencv_images"].as<bool>();
            config.show_opencv_images = params["pub_images"].as<bool>();
            config.show_opencv_images = params["interpolate"].as<bool>();
            config.nwindows = params["nwindows"].as<uint8_t>();
            config.min_pixels = params["min_pixels"].as<uint8_t>();
            config.x_shift_factor = params["x_shift_factor"].as<float>();
            config.window_width_factor = params["window_width_factor"].as<float>();
            config.lane_distance_factor = params["lane_distance_factor"].as<float>();
            config.safe_lane_distance_factor = params["safe_lane_distance_factor"].as<float>();
            config.left_init_factor = params["left_init_factor"].as<float>();
            config.right_init_factor = params["right_init_factor"].as<float>();
            config.valid_curve_factor = params["valid_curve_factor"].as<float>();
            config.valid_band = params["valid_band"].as<float>();
            config.first_iters_each_side = params["first_iters_each_side"].as<uint8_t>();
            config.iter_each_side = params["iter_each_side"].as<uint8_t>();
            config.binary_adaptive_box = params["binary_adaptive_box"].as<uint16_t>();
            config.binary_adaptive_subtract = params["binary_adaptive_subtract"].as<int16_t>();
            config.num_threads = params["num_threads"].as<uint8_t>();
            config.camcal_yaml = params["camcal_yaml"].as<std::string>();
        }
    } catch (const YAML::Exception& e) {
        std::cerr << "Error loading YAML configuration: " << e.what() << std::endl;
        throw std::runtime_error("Failed to load configuration from YAML.");
    }
    post_init_lane_detection_params(&config);
}


LaneDetectionParams default_config() {
    static LaneDetectionParams config;
    std::string config_path = "${workspaceFolder}/src/1_sense/lane_detection/config/default_config.yaml";
    load_config_from_yaml(config_path, config);
    return config;
}


LaneDetectionParams debug_config() {
    static LaneDetectionParams config;
    std::string config_path = "${workspaceFolder}/src/1_sense/lane_detection/config/debug_config.yaml";
    load_config_from_yaml(config_path, config);
    return config;
}

LaneDetectionParams ros_debug_config() {
    static LaneDetectionParams config;
    std::string config_path = "${workspaceFolder}/src/1_sense/lane_detection/config/ros_debug_config.yaml";
    load_config_from_yaml(config_path, config);
    return config;
}