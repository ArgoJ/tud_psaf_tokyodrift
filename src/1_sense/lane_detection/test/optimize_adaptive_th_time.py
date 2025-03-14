import cv2
import numpy as np

def process_image(colored_path, label_path):
    # Load the images
    colored_img = cv2.imread(colored_path, cv2.IMREAD_COLOR)
    label_img = cv2.imread(label_path, cv2.IMREAD_GRAYSCALE)

    if colored_img is None or label_img is None:
        raise FileNotFoundError("One or both of the images could not be loaded. Check the file paths.")

    if colored_img.shape[:2] != label_img.shape[:2]:
        raise ValueError("The dimensions of the colored image and the label image do not match.")

    _, binary_label = cv2.threshold(label_img, 1, 255, cv2.THRESH_BINARY)
    x, y, w, h = cv2.boundingRect(binary_label)

    # Crop the region of interest (ROI) in both images
    roi_colored = colored_img[y:y+h, x:x+w]
    roi_label = label_img[y:y+h, x:x+w]

    # Apply adaptive thresholding to the cropped colored image
    gray_roi = cv2.cvtColor(roi_colored, cv2.COLOR_BGR2GRAY)
    adaptive_thresh = cv2.adaptiveThreshold(
        gray_roi,
        maxValue=255,
        adaptiveMethod=cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
        thresholdType=cv2.THRESH_BINARY,
        blockSize=11,
        C=2
    )

    overlap = cv2.bitwise_and(adaptive_thresh, roi_label)

    white_pixels_overlap = np.sum(overlap == 255)
    total_white_pixels_label = np.sum(roi_label == 255)
    overlap_ratio = white_pixels_overlap / total_white_pixels_label if total_white_pixels_label > 0 else 0

    print(f"Overlap ratio: {overlap_ratio:.2%}")

    cv2.imshow("Original ROI", roi_colored)
    cv2.imshow("Adaptive Threshold", adaptive_thresh)
    cv2.imshow("Label ROI", roi_label)
    cv2.imshow("Overlap", overlap)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

    return overlap_ratio

# Example usage
colored_image_path = "path_to_colored_image.jpg"
label_image_path = "path_to_label_image.jpg"
process_image(colored_image_path, label_image_path)
