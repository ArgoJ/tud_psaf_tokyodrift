#include "lane_detection_config.h"



void post_init_lane_detection_params(LaneDetectionParams* config) {
    if (config == nullptr) return;

    std::string cwd = fs::current_path().string(); 
    if (cwd.find("build") != std::string::npos) {
        // If we are in a 'build' directory, go two levels up (to get back to 'src')
        cwd = fs::current_path().parent_path().parent_path().string();
    }
    std::string yaml_file_path = cwd + "/" + config->camcal_yaml;
    std::cout << "Using Camcal Yaml File: " << yaml_file_path << std::endl;

    YAML::Node config_node = YAML::LoadFile(yaml_file_path);

    config->bird_view_width = config_node["camcal"]["ipm"]["ego_width_px"].as<int>();
    config->bird_view_height = config_node["camcal"]["ipm"]["ego_height_px"].as<int>();
    config->px2m = 1e-3f / config_node["camcal"]["ipm"]["ego_scale_mm2px"].as<float>();
    config->x0_m = config_node["camcal"]["ego_target"]["egorange_x_mm"][0].as<float>() * 1e-3f;
    
    // get transformation matrix
    std::string H_string = config_node["camcal"]["ipm"]["H_rowmajor"].as<std::string>();
    config->transformation_matrix = parse_transformation_matrix(H_string);

    std::string roi_str = config_node["camcal"]["roi"]["img_roi_px"].as<std::string>();
    std::vector<cv::Point> roi = parse_roi(roi_str);
    config->roi_height = get_max_roi_height(roi);

    // Set defaults
    config->window_width = static_cast<uint16_t>(static_cast<float>(config->bird_view_width)* config->window_width_factor);
    config->window_height = static_cast<uint16_t>(static_cast<float>(config->bird_view_height) / config->nwindows);
    config->x_shift = static_cast<uint16_t>(static_cast<float>(config->bird_view_width) * config->x_shift_factor);
    config->x_shift_next = static_cast<uint16_t>(static_cast<float>(config->bird_view_width) * config->x_shift_next_factor);
    config->lane_distance = static_cast<uint16_t>(static_cast<float>(config->bird_view_width) * config->lane_distance_factor);
    config->safe_lane_distance = static_cast<uint16_t>(static_cast<float>(config->lane_distance) * config->safe_lane_distance_factor);

    update_iters(config);

    if (config->left_init_x == 0) {
        config->left_init_x = static_cast<uint16_t>(static_cast<float>(config->bird_view_width) * config->left_init_factor);
    }
    if (config->right_init_x == 0) {
        config->right_init_x = static_cast<uint16_t>(static_cast<float>(config->bird_view_width) * config->right_init_factor);
    }

    // check for monitor presens
    if (!HAS_DISPLAY) {
        config->show_opencv_images = false;
    }

    // check for maximal threads
    config->num_threads = std::min(static_cast<uint8_t>(omp_get_max_threads()), config->num_threads);
}


cv::Mat_<double> parse_transformation_matrix(const std::string& M_string) {
    std::string clean_M_string = M_string;
    clean_M_string.erase(std::remove(clean_M_string.begin(), clean_M_string.end(), '['), clean_M_string.end());
    clean_M_string.erase(std::remove(clean_M_string.begin(), clean_M_string.end(), ']'), clean_M_string.end());

    std::vector<double> values;
    std::istringstream iss(clean_M_string);
    std::string num;
    while (std::getline(iss, num, ',')) {
        values.push_back(std::stod(num));
    }

    if (values.size() != 9) {
        throw std::runtime_error("Invalid transformation matrix format");
    }

    return (cv::Mat_<double>(3, 3) <<
        values[0], values[1], values[2],
        values[3], values[4], values[5],
        values[6], values[7], values[8]
    );
}


std::vector<cv::Point> parse_roi(const std::string& str) {
    std::vector<cv::Point> points;
    std::string cleaned_str = str.substr(1, str.size() - 2);
    
    std::stringstream ss(cleaned_str);
    std::string point_str;
    
    while (std::getline(ss, point_str , ']')) {
        point_str.erase(std::remove_if(point_str.begin(), point_str.end(), ::isspace), point_str.end());

        if (point_str.empty()) continue;
        
        point_str.erase(0, point_str.find_first_of("[") + 1);
        std::stringstream point_stream(point_str);
        std::string num;
        double x, y;

        // Extract x and y values, handling spaces
        if (std::getline(point_stream, num, ',')) {
            x = std::stod(num);
        }
        if (std::getline(point_stream, num, ',')) {
            y = std::stod(num);
        }
        
        points.push_back(cv::Point(static_cast<int>(x), static_cast<int>(y)));
    }
    return points;
}


uint16_t get_max_roi_height(std::vector<cv::Point> points) {
    auto min_point = std::min_element(
        points.begin(), points.end(), 
        [](const cv::Point& a, const cv::Point& b) {
            return a.y < b.y; // Compare y values
        }
    );
    return static_cast<uint16_t>(min_point != points.end() ? min_point->y : std::numeric_limits<uint16_t>::infinity());
}


void update_iters(LaneDetectionParams* config) {
    // Example logic for generating iterators based on config
    config->left_iters = generate_iters(config->iter_each_side, CurrentLine::LEFT_LINE);
    config->right_iters = generate_iters(config->iter_each_side, CurrentLine::RIGHT_LINE);
    config->notfound_left_iters = generate_iters(config->first_iters_each_side, CurrentLine::LEFT_LINE);
    config->notfound_right_iters = generate_iters(config->first_iters_each_side, CurrentLine::RIGHT_LINE);
}


std::vector<int16_t> generate_iters(int16_t iters_each_side, CurrentLine current_lane) {
    //if you want the left iters, use LEFT_LINE, else use RIGHT_LINE
    std::vector<int16_t> result = {0};
    for (int16_t i = 1; i <= iters_each_side; ++i) {
        if (current_lane == CurrentLine::RIGHT_LINE) {
            result.push_back(i);  
        }
        result.push_back(-i);
        if (current_lane == CurrentLine::LEFT_LINE) {
            result.push_back(i);  
        }
    }
    
    return result;
}


std::ostream& operator<<(std::ostream& os, const LaneDetectionParams& config) {
    os << "LaneDetectionParams { " << std::endl;
    os << "  show_opencv_images: " << config.show_opencv_images << std::endl;
    os << "  publish_images: " << config.pub_images << std::endl;
    os << "  nwindows: " << (int)config.nwindows << std::endl;
    os << "  min_pixels: " << (int)config.min_pixels << std::endl;
    os << "  x_shift_factor: " << config.x_shift_factor << std::endl;
    os << "  window_width_factor: " << config.window_width_factor << std::endl;
    os << "  lane_distance_factor: " << config.lane_distance_factor << std::endl;
    os << "  safe_lane_distance_factor: " << config.safe_lane_distance_factor << std::endl;
    os << "  left_init_factor: " << config.left_init_factor << std::endl;
    os << "  right_init_factor: " << config.right_init_factor << std::endl;
    os << "  valid_curve_factor: " << config.valid_curve_factor << std::endl;
    os << "  first_iters_each_side: " << (int)config.first_iters_each_side << std::endl;
    os << "  iter_each_side: " << (int)config.iter_each_side << std::endl;
    os << "  bird_view_width: " << (int)config.bird_view_width << std::endl;
    os << "  bird_view_height: " << (int)config.bird_view_height << std::endl;
    os << "  mm2px: " << (float)config.px2m << std::endl;
    os << "  transformation_matrix: " << (cv::Mat)config.transformation_matrix << std::endl;
    os << "  binary_adaptive_box: " << (int)config.binary_adaptive_box << std::endl;
    os << "  binary_adaptive_subtract: " << (int)config.binary_adaptive_subtract << std::endl;
    os << "  left_init_x: " << (int)config.left_init_x << std::endl;
    os << "  right_init_x: " << (int)config.right_init_x << std::endl;
    os << "  window_width: " << (int)config.window_width << std::endl;
    os << "  window_height: " << (int)config.window_height << std::endl;
    os << "  x_shift: " << (int)config.x_shift << std::endl;
    os << "  lane_distance: " << (int)config.lane_distance << std::endl;
    os << "  safe_lane_distance: " << (int)config.safe_lane_distance << std::endl;
    os << "}";

    return os;
}