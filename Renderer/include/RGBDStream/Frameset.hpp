#pragma once

#include "RGBDStream.hpp"
#include "opencv2/opencv.hpp"

#include <vector>
#include <utility>
#include <variant>
#include <memory>

static const int RGB_CHANNELS = 3;

enum class StreamType : int {
	Color = 0,
	Depth = 1
}; // I want to keep this implementation open to the addition alternative streams like IR.

struct RGBData {
	cv::Mat image;
	int width() const { return image.cols; }
	int height() const { return image.rows; }
};

struct DepthData {
	std::vector<uint16_t> data;
	int width;
	int height;
	double scale;
	int stride() const { return width * sizeof(uint16_t); }
};

class Frame {
public:
	std::variant<RGBData, DepthData> data;

	Frame(RGBData&& data) : data(std::move(data)) {};
	Frame(DepthData&& data) : data(std::move(data)) {};

	const RGBData* AsColor() const {
		return std::get_if<RGBData>(&data);
	}
	const DepthData* AsDepth() const {
		return std::get_if<DepthData>(&data);
	}
};

class Frameset {
	std::vector<std::pair<StreamType, std::shared_ptr<Frame>>> frames{};
public:
	void AddColor(cv::Mat image) {
		frames.emplace_back(StreamType::Color, std::make_shared<Frame>(RGBData{ std::move(image) }));
	}
	void AddDepth(std::vector<uint16_t>&& data, int w, int h, double scale) {
		frames.emplace_back(StreamType::Depth, std::make_shared<Frame>(DepthData{ std::move(data), w, h, scale }));
	}
	size_t Size() const {
		return this->frames.size();
	}
	const std::shared_ptr<Frame> GetFirst(StreamType type) const {
		for (const auto& f : this->frames) {
			if (f.first == type)
				return f.second;
		}
		return nullptr;
	}
};