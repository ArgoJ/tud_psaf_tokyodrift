#include "start_detector.h"
#include <iostream>
#include <time.h>
#include <math.h>

StartDetector::StartDetector() {
    std::cout << "start detector pml. " << std::endl;
}

bool StartDetector::is_red_square(cv::Mat& window, uint8_t window_size, uint8_t pixels_threshold){
    int number_of_white_pixels = cv::countNonZero(window);
    float red_percentage = ((1.0 * number_of_white_pixels) / (window_size * window_size)) * 100; //if number of red pixels > certain threshold -> it is a red square.
    return red_percentage >= pixels_threshold;
}

int StartDetector::get_number_of_red_squares(const cv::Mat& binary_image, uint8_t window_size, uint8_t pixels_threshold, uint8_t step_size){
    int height = binary_image.rows;
    int width = binary_image.cols; 
    int number_of_red_squares = 0;
    for (int y = 0; y < height - window_size; y += step_size){
        for (int x = 0; x < width - window_size; x += step_size){
            //get window
            cv::Rect roi(x, y, window_size, window_size);
            cv::Mat window = binary_image(roi);

            //test if it is a red square. if so, increment number of red squares!
            number_of_red_squares += is_red_square(window, window_size, pixels_threshold);
        }
    } 

    return number_of_red_squares;
}

const cv::Mat StartDetector::convert_image(const cv::Mat& image_bgr){
    std::vector<int> lower_red1 = {0, 120, 70};
    std::vector<int> upper_red1 = {10, 255, 255};
    std::vector<int> lower_red2 = {170, 120, 70};
    std::vector<int> upper_red2 = {180, 255, 255};

    cv::Mat image_hsv;
    cv::cvtColor(image_bgr, image_hsv, cv::COLOR_BGR2HSV); //Converting image to hsv to filter for color
    cv::Mat image_blur;
    cv::GaussianBlur(image_hsv, image_blur, cv::Size(5, 5), 0); //Using gausean blur to detect egdes better

    cv::Mat mask1;
    cv::inRange(image_blur, lower_red1, upper_red1, mask1); //lower red mask
    cv::Mat mask2;
    cv::inRange(image_blur, lower_red2, upper_red2, mask2);//upper red mask

    cv::Mat red_mask;
    cv::bitwise_or(mask1, mask2, red_mask);//combined red mask

    return red_mask; //because the red mask itslef is already a binary representation of the red pixels in our image, we can return the red mask directly.
}
bool StartDetector::is_stop_sign_still_visible(const cv::Mat& binary_image, uint8_t window_size, uint8_t pixels_threshold, uint8_t squares_threshold, uint8_t step_size){
    int number_of_red_squares = get_number_of_red_squares(binary_image, window_size, pixels_threshold, step_size);
    if (number_of_red_squares > squares_threshold){
        //std::cout << "visible" << std::endl;
        return true;
    } 
    else{
        //std::cout << "NOOOOOOOOOOOOOOOOOT visible" << std::endl; 
        return false;
    }
}

