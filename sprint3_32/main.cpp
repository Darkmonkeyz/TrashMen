#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

int main()
{
    // Read the two images
    cv::Mat image1 = cv::imread("../map3.png");
    cv::Mat image2 = cv::imread("../map4.png");

    // Check if images were successfully loaded
    if (image1.empty() || image2.empty()) {
        std::cerr << "Could not open or find the images!" << std::endl;
        return -1;
    }

    
    cv::addWeighted(image1, 0.7, image2, 0.3, 0, image1);

    // Display the result
    cv::imshow("Overlay Result", image1);


    // Wait for a key press and close the window
    cv::waitKey(0);
    cv::destroyAllWindows();

    return 0;
}