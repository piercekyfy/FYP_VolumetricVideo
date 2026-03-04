#pragma once

#include <vector>

#include "RGBDStream/RGBDStream.hpp"

#include "opencv2/core.hpp"
#include "opencv2/stereo.hpp"

struct StereoPoseResult {
	bool Success;
	cv::Mat R, T, E, F;
	double Error;
	int SuccessfulSamples;
};

class StereoCalibrator {
public:
	StereoCalibrator(int boardWidth, int boardHeight, double cellSize, int imageWidth, int imageHeight)
		: boardWidth(boardWidth), boardHeight(boardHeight), cellSize(cellSize),
		imageWidth(imageWidth), imageHeight(imageHeight)
	{
	}

	bool Process(Frame* R, Frame* L) {
		auto rgbR = R->AsColor();
		auto rgbL = L->AsColor();

		if (rgbR == nullptr || rgbL == nullptr)
			throw std::exception("Cannot estimate pose from non-rgb frame.");

		return Feed(rgbR, rgbL);
	}

	StereoPoseResult GetPose(Intrinsics intrR, Intrinsics intrL) {
		return Calibrate(intrR, intrL);
	}

	const int SampleSize() const {
		return this->sampleSize;
	}

private:
	int boardWidth;
	int boardHeight;
	double cellSize;
	int imageWidth{};
	int imageHeight{};
	std::vector<std::vector<cv::Point3f>> objPoints{};
	std::vector<std::vector<cv::Point2f>> imgPointsR, imgPointsL{};
	int sampleSize{ 0 };

	const bool Feed(const RGBData* R, const RGBData* L) {
		cv::Mat imageR{ R->image.clone() };
		cv::Mat imageL{ L->image.clone() };
		cv::cvtColor(imageR, imageR, cv::COLOR_RGB2GRAY);
		cv::cvtColor(imageL, imageL, cv::COLOR_RGB2GRAY);

		std::vector<cv::Point2f> pointsR{};
		std::vector<cv::Point2f> pointsL{};
		if (!FindCheckerboard(imageR, pointsR)) {
			std::cout << "Failed to find source0 checkerboard!";
			return false;
		}
		if (!FindCheckerboard(imageL, pointsL)) {
			std::cout << "Failed to find source1 checkerboard!";
			return false;
		}

		std::vector<cv::Point3f> localObjectPoints{};
		for (size_t i = 0; i < boardHeight; i++) {
			for (size_t j = 0; j < boardWidth; j++) {
				localObjectPoints.emplace_back(j * cellSize, i * cellSize, 0.0f);
			}
		}

		objPoints.push_back(localObjectPoints);
		imgPointsR.push_back(pointsR);
		imgPointsL.push_back(pointsL);

		sampleSize++;

		return true;
	}

	const StereoPoseResult Calibrate(Intrinsics intrR, Intrinsics intrL) const {
		if (sampleSize <= 0)
			return { false };

		cv::Mat R, T, E, F;

		double error = cv::stereoCalibrate(
			objPoints,
			imgPointsR,
			imgPointsL,
			GetIntrinsicsMat(intrR),
			GetCoeffsMat(intrR),
			GetIntrinsicsMat(intrL),
			GetCoeffsMat(intrL),
			cv::Size{ imageWidth, imageHeight }, R, T, E, F, cv::CALIB_FIX_INTRINSIC, cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 100, 1e-5));

		return { true, R, T, E, F, error, sampleSize }; // Rotation between, translation between, essential matrix, fundament matrix (pixel L -> pixel R)
	}

	const bool FindCheckerboard(cv::Mat image, std::vector<cv::Point2f>& corners) const {
		bool foundCorners = cv::findChessboardCorners(image, cv::Size{ boardWidth, boardHeight }, corners);

		if (!foundCorners) { return false; };

		cv::cornerSubPix(image, corners, cv::Size{ 11, 11 }, cv::Size{ -1, -1 }, cv::TermCriteria{ cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 30, 0.001 });

		return true;
	}

	const cv::Mat GetIntrinsicsMat(Intrinsics intrinsics) const {
		return (cv::Mat_<double>(3, 3) << intrinsics.fx, 0, intrinsics.ppx, 0, intrinsics.fy, intrinsics.ppy, 0, 0, 1);
	}

	const cv::Mat GetCoeffsMat(Intrinsics intrinsics) const {
		return (cv::Mat_<double>(1, 5) << intrinsics.coeffs[0], intrinsics.coeffs[1], intrinsics.coeffs[2], intrinsics.coeffs[3], intrinsics.coeffs[4]);
	}
};