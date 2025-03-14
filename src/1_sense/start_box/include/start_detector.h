#include <opencv2/opencv.hpp>

#ifndef TOKYODRIFT_START_DETECTOR_H
#define TOKYODRIFT_START_DETECTOR_H

class StartDetector{
    public:
        StartDetector(); //Constructor
    /**
     * Checks whether there is a stop sign or not. 
     * Note: this method uses a BINARY IMAGE (filtered for red value) and a sliding window approach.
     * 
     * @param binary_image The binary image to be checked.
     * @param window_size The side length (x or y) of the each individual square window for the window approach
     * @param pixels_threshold The threshold of pixels that have to be red within a square to count as a red square
     * @param squares_threshold The number of red squares the image must contain in order to detect a stop sign
     * @param step_size The step size for the sliding window approach
     * @return true if the window contains a stop sign.
     */
        bool is_stop_sign_still_visible(const cv::Mat& binary_image, uint8_t window_size, uint8_t pixels_threshold, uint8_t squares_threshold, uint8_t step_size);

    /**
     * Converts the BGR image to a binary image that contains only the red pixels.
     *  
     * @param image_bgr The image to be converted.
     * @return the converted binary image.
     */    
        const cv::Mat convert_image(const cv::Mat& image_bgr);
    private:
    /**
     * Checks if a given window consists of more than this->config.pixels_threshold red pixels
     * 
     * @param window The window to be checked.
     * @param window_size The side length (x or y) of the each individual square window for the window approach
     * @param pixels_threshold The threshold of pixels that have to be red within a square to count as a red square
     * @return true if the window consists of more than this->config.pixels_threshold red pixels.
     */
        bool is_red_square(cv::Mat& window, uint8_t window_size, uint8_t pixels_threshold);

    /**
     * Counts the number of red squares in the given binary image.
     * 
     * @param binary_image The window to be checked.
     * @param window_size The side length (x or y) of the each individual square window for the window approach
     * @param pixels_threshold The threshold of pixels that have to be red within a square to count as a red square
     * @param step_size The step size for the sliding window approach
     * @return true if the window consists of more than this->config.pixels_threshold red pixels.
     */
        int get_number_of_red_squares(const cv::Mat& binary_image, uint8_t window_size, uint8_t pixels_threshold, uint8_t step_size); 
};

#endif