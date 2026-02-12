#include <iostream>

#include "opencv2/opencv.hpp"

#include "glcommon.hpp"
#include "RGBDStream/FileRGBDStream.hpp"


int main() {
    std::string relPath = getPathWindows();

    RGBDStream::FileRGBDStream cTarget{ relPath + "\\frames\\239622300610" };
    RGBDStream::FileRGBDStream cSource{ relPath + "\\frames\\241122306275" };

    auto intrT = cTarget.GetDescription().GetFirst(StreamType::Color)->get().intrinsics;
    auto intrS = cSource.GetDescription().GetFirst(StreamType::Color)->get().intrinsics;

    cv::Mat matrixTarget = (cv::Mat_<double>(3, 3) << intrT.fx, 0, intrT.ppx, 0, intrT.fy, intrT.ppy, 0, 0, 1);
    cv::Mat matrixSource = (cv::Mat_<double>(3, 3) << intrS.fx, 0, intrS.ppx, 0, intrS.fy, intrS.ppy, 0, 0, 1);
    cv::Mat coeffTarget = (cv::Mat_<double>(1, 5) << intrT.coeffs[0], intrT.coeffs[1], intrT.coeffs[2], intrT.coeffs[3], intrT.coeffs[4]);
    cv::Mat coeffSource = (cv::Mat_<double>(1, 5) << intrS.coeffs[0], intrS.coeffs[1], intrS.coeffs[2], intrS.coeffs[3], intrS.coeffs[4]);

    auto fsTarget = cTarget.WaitForFrames();
    auto fsSource = cSource.WaitForFrames();

    cv::Mat bgrTarget{ fsTarget->GetFirst(StreamType::Color)->AsColor()->image.clone() };
    cv::Mat bgrSource{ fsSource->GetFirst(StreamType::Color)->AsColor()->image.clone() };

    cv::cvtColor(bgrTarget, bgrTarget, cv::COLOR_RGB2BGR);
    cv::cvtColor(bgrSource, bgrSource, cv::COLOR_RGB2BGR);

    cv::Mat greyTarget{};
    cv::Mat greySource{};

    cv::cvtColor(bgrTarget.clone(), greyTarget, cv::COLOR_BGR2GRAY);
    cv::cvtColor(bgrSource.clone(), greySource, cv::COLOR_BGR2GRAY);

    cv::Size boardSize{ 6, 9 };
    double cellSize{ 0.253 };

    std::vector<cv::Point3f> objectPoints;

    for (size_t i = 0; i < boardSize.height; i++) {
        for (size_t j = 0; j < boardSize.width; j++) {
            objectPoints.emplace_back(j * cellSize, i * cellSize, 0.0f); // Fixed 'real-world' coordinates of the chessboard.
        }
    }

    std::vector<cv::Point2f> cornersTarget;
    std::vector<cv::Point2f> cornersSource;

    bool foundTarget = cv::findChessboardCorners(greyTarget, boardSize, cornersTarget);
    bool foundSource = cv::findChessboardCorners(greySource, boardSize, cornersSource);

    if (!foundTarget || !foundSource) {
        std::cout << "Failed to find chessboard in " << (foundTarget ? "source image" : (foundSource ? "target image" : "both images")) << '\n'; // writing confusing ternaries is my passion
        return 1;
    }

    cv::cornerSubPix(greyTarget, cornersTarget, cv::Size{ 11, 11 }, cv::Size{ -1, -1 }, cv::TermCriteria{ cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 30, 0.001 });
    cv::cornerSubPix(greySource, cornersSource, cv::Size{ 11, 11 }, cv::Size{ -1, -1 }, cv::TermCriteria{ cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 30, 0.001 });

    cv::Mat rotTarget, transTarget;
    bool solvedTarget = cv::solvePnP(objectPoints, cornersTarget, matrixTarget, coeffTarget, rotTarget, transTarget);

    cv::Rodrigues(rotTarget, rotTarget);
    
    // Project object points to image (to see if PnP solve worked)

    std::vector<cv::Point2f> projectedPointsTarget{};
    cv::projectPoints(objectPoints, rotTarget, transTarget, matrixTarget, coeffTarget, projectedPointsTarget);

    for (size_t i = 0; i < cornersTarget.size(); i++) {
        cv::circle(bgrTarget, cornersTarget[i], 4, { 0, 255, 0 }, -1); // detected (green)
        cv::circle(bgrTarget, projectedPointsTarget[i], 4, { 0, 0, 255 }, -1); // projected (red)
    }

    cv::imshow("PnP on Target", bgrTarget);
    cv::waitKey(0);
    cv::destroyAllWindows();

    return 1;
}