#pragma once

#include <vector>
#include <iostream>

#include "opencv2/core.hpp"
#include "opencv2/aruco.hpp"
#include "opencv2/objdetect.hpp"

#include "RGBDStream/Frameset.hpp"

struct CalibrationPair {
	std::vector<cv::Point3f> objectPoints{};
	std::vector<cv::Point2f> sourceImagePoints{};
	std::vector<cv::Point2f> targetImagePoints{};
};

struct StereoCalibrationResult {
	bool Success;
	cv::Mat R, T, E, F;
	double Error;
};

class CharucoCalibration {
public:
	CharucoCalibration(int xSize, int ySize, double squareSize, double markerSize, cv::aruco::PredefinedDictionaryType dict = cv::aruco::DICT_5X5_100, int minCorners = 6)
		: board(cv::Size{xSize, ySize}, squareSize, markerSize, cv::aruco::getPredefinedDictionary(dict)), detector(this->board), boardObjectPoints(this->board.getChessboardCorners()), minCorners(minCorners) {
		
	}
	bool Feed(Frame* sourceFrame, Frame* targetFrame) {
		cv::Mat sourceImg{ sourceFrame->AsColor()->image.clone() };
		cv::Mat targetImg{ targetFrame->AsColor()->image.clone() };

		cv::imshow("debug", sourceImg);
		cv::imshow("debug2", targetImg);

		std::vector<cv::Point2f> sourceCorners{};
		std::vector<int> sourceIds{};

		std::vector<cv::Point2f> targetCorners{};
		std::vector<int> targetIds{};

		this->detector.detectBoard(sourceImg, sourceCorners, sourceIds);
		this->detector.detectBoard(targetImg, targetCorners, targetIds);

		if (sourceCorners.size() < this->minCorners ||targetCorners.size() < this->minCorners)
			return false;

		cv::Rect sourceBounds = cv::boundingRect(sourceCorners); // If corners are too clustered they're likely bad.
		cv::Rect targetBounds = cv::boundingRect(targetCorners);
		if (sourceBounds.width < 20 || sourceBounds.height < 20 || targetBounds.width < 20 || targetBounds.height < 20) {
			std::cout << "Threw away bad pair \n";
			return false;
		}

		CalibrationPair pair{};
		pair.objectPoints.reserve(sourceIds.size());
		pair.sourceImagePoints.reserve(sourceIds.size());
		pair.targetImagePoints.reserve(sourceIds.size());

		std::unordered_map<int, int> targetLookup{};
		targetLookup.reserve(targetIds.size());
		for (int i = 0; i < targetIds.size(); i++) {
			targetLookup[targetIds[i]] = i;
		}

		for (int i = 0; i < sourceIds.size(); i++) {
			auto tItr = targetLookup.find(sourceIds[i]);
			if (tItr == targetLookup.end())
				continue;

			pair.objectPoints.push_back(this->boardObjectPoints[sourceIds[i]]);
			pair.sourceImagePoints.push_back(sourceCorners[i]);
			pair.targetImagePoints.push_back(targetCorners[tItr->second]);
		}

		if (pair.objectPoints.size() < this->minCorners)
			return false;

		pairs.push_back(std::move(pair));

		return true;
	}
	const StereoCalibrationResult Calibrate(const Intrinsics& sourceIntrinsics, const Intrinsics& targetIntrinsics) const {
		if (pairs.size() <= 0)
			return { false };

		std::vector<std::vector<cv::Point3f>> objectPoints;
		std::vector<std::vector<cv::Point2f>> sourceImagePoints;
		std::vector<std::vector<cv::Point2f>> targetImagePoints;
		objectPoints.reserve(pairs.size());
		sourceImagePoints.reserve(pairs.size());
		targetImagePoints.reserve(pairs.size());

		for (const auto& pair : this->pairs) {
			objectPoints.push_back(pair.objectPoints);
			sourceImagePoints.push_back(pair.sourceImagePoints);
			targetImagePoints.push_back(pair.targetImagePoints);
		}

		cv::Mat R, T, E, F;
		int flags = cv::CALIB_FIX_INTRINSIC;

		double error = cv::stereoCalibrate(
			objectPoints,
			sourceImagePoints,
			targetImagePoints,
			GetIntrinsicsMat(sourceIntrinsics),
			GetCoeffsMat(sourceIntrinsics),
			GetIntrinsicsMat(targetIntrinsics),
			GetCoeffsMat(targetIntrinsics),
			cv::Size{ sourceIntrinsics.width, sourceIntrinsics.height },
			R, T, E, F,
			flags,
			cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 100, 1e-5));

		return { true, R, T, E, F, error };
	}
	int ValidPairs() {
		return pairs.size();
	}
private:
	cv::aruco::CharucoBoard board;
	cv::aruco::CharucoDetector detector;
	std::vector<cv::Point3f> boardObjectPoints;
	int minCorners{ 1 };
	std::vector<CalibrationPair> pairs{};

	const cv::Mat GetIntrinsicsMat(const Intrinsics& intrinsics) const {
		return (cv::Mat_<double>(3, 3) << intrinsics.fx, 0, intrinsics.ppx, 0, intrinsics.fy, intrinsics.ppy, 0, 0, 1);
	}

	const cv::Mat GetCoeffsMat(const Intrinsics& intrinsics) const {
		return (cv::Mat_<double>(1, 5) << intrinsics.coeffs[0], intrinsics.coeffs[1], intrinsics.coeffs[2], intrinsics.coeffs[3], intrinsics.coeffs[4]);
	}
};