#include "glm/glm.hpp"
#include "glad/glad.h"
#include "GLFW/glfw3.h"
#include "opencv2/opencv.hpp"

#include <iostream>

int main() {

    cv::Mat mat = (cv::Mat_<float>(3, 3) << 1, 2, 3,
        4, 5, 6,
        7, 8, 9);

    return 0;
}