#ifndef TOKYODRIFT_INTERPOLATION_H
#define TOKYODRIFT_INTERPOLATION_H

#include <vector>
#include <opencv2/opencv.hpp>
#include "lane_detection_helper.hpp"

class Interpolation {

public:
    std::vector<cv::Point> interpolate_line_points(const std::vector<cv::Point>& lane, const int height) {
        if (lane.empty()){
            return {};
        }
        std::vector<int> x_values, y_values, x_final;
        std::transform(lane.begin(), lane.end(), std::back_inserter(x_values), [](const cv::Point& point) { return point.x; });
        std::transform(lane.begin(), lane.end(), std::back_inserter(y_values), [](const cv::Point& point) { return point.y; });

        auto y_start = *std::max_element(y_values.begin(), y_values.end());
        auto y_start_diff = height - y_start;

        auto y_new = linspace(0, height - 1, height);

        std::vector<int> y_new_sublist(y_new.begin(), y_new.begin() + y_start);
        std::reverse(y_values.begin(), y_values.end());
        std::reverse(x_values.begin(), x_values.end());
        auto x_new = linear_interp(y_new_sublist, y_values, x_values); 
        //NOTE that order of values of x_new is inverse to x_values (which makes sense as y_new is also the other way around (from 0 to max_height))
        std::reverse(y_values.begin(), y_values.end());
        std::reverse(x_values.begin(), x_values.end());

        std::reverse(x_new.begin(), x_new.end());
        std::reverse(y_new.begin(), y_new.end());

        if (y_start_diff != 0){
            auto start_grad = discrete_gradient(cv::Point(x_values.front(), y_values.front()), cv::Point(x_values[1], y_values[1]));
            auto x_new_append = linspace(x_values.front(), (int) (x_values.front() + y_start_diff * start_grad), y_start_diff);
            std::reverse(x_new_append.begin(), x_new_append.end());
            std::move(x_new_append.begin(), x_new_append.end(), std::back_inserter(x_final));
            std::move(x_new.begin(), x_new.end(), std::back_inserter(x_final));
        }
        return stack_and_transpose(x_final, y_new);
    }

private:    
    std::vector<int> linspace(int start, int stop, int num) {
        std::vector<int> result;

        //circumvent div by 0
        if (num == 1) {
            result.push_back(start);
            return result;
        }
        
        float step = (static_cast<float>(stop - start) / static_cast<float>(num - 1));
        for (int i = 0; i < num; ++i) {
            int new_value = static_cast<int> (start + i * step);
            result.push_back(new_value);
        }

        return result;
    }

    std::vector<int> linear_interp(const std::vector<int>& x, const std::vector<int>& xp, const std::vector<int>& fp) {
        std::vector<int> result;

        for (int val : x) {
            if (val <= xp.front()) {
                // Below range: use the first value
                result.push_back(fp.front());
            } else if (val >= xp.back()) {
                // Above range: use the last value
                result.push_back(fp.back());
            } else {
                // Find the interval [xp[i], xp[i+1]] where val lies
                for (size_t i = 0; i < xp.size() - 1; ++i) {
                    if (val >= xp[i] && val <= xp[i + 1]) {
                        // Integer interpolation formula
                        int t = (val - xp[i]) * (fp[i + 1] - fp[i]) / (xp[i + 1] - xp[i]);
                        result.push_back(fp[i] + t);
                        break;
                    }
                }
            }
        }

        return result;
    }


    std::vector<cv::Point> stack_and_transpose(const std::vector<int>& x_new, const std::vector<int>& y_new) {
        std::vector<cv::Point> result;

        // Assuming both vectors x_new and y_new are of the same size
        size_t n = x_new.size();

        // Create pairs and store them in the result
        for (size_t i = 0; i < n; ++i) {
            result.push_back(cv::Point(x_new[i], y_new[i]));
        }

        return result;
    }
};

#endif //TOKYODRIFT_INTERPOLATION_H