#pragma once

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

struct Intrinsics {
	int width;
	int height;
	int ppx;
	int ppy;
	int fx;
	int fy;
	int coeffs[5];
};

struct Stream {
	StreamType type;
	int fps;
	int bpp;
	Intrinsics intrinsics;
};

struct FrameDescription { // Type name is incorrect. Should be 'FramesetDescription' (but it is too cumbersome to change rn)
	std::string serial;
	double depthScale{ 0.0 };
	std::vector<Stream> streams{};
public:
	std::optional<std::reference_wrapper<const Stream>> GetFirst(StreamType type) const {
		for (const auto& s : this->streams) {
			if (s.type == type)
				return std::cref(s);
		}
		return std::nullopt;
	}
};

class Frame {
public:
	std::variant<RGBData, DepthData> data;

	Frame(RGBData&& data, Stream description) : data(std::move(data)), description(description) {};
	Frame(DepthData&& data, Stream description) : data(std::move(data)), description(description) {};

	const RGBData* AsColor() const {
		return std::get_if<RGBData>(&data);
	}
	const DepthData* AsDepth() const {
		return std::get_if<DepthData>(&data);
	}
	const Stream GetDescription() {
		return this->description;
	}
private:
	Stream description{};
};

class Frameset {
private:
	FrameDescription description;
public:
	Frameset(FrameDescription description) : description(description) {}
	std::vector<std::pair<StreamType, std::shared_ptr<Frame>>> frames{}; // TODO: make private accessor
	void AddColor(cv::Mat image, Stream description) {
		frames.emplace_back(StreamType::Color, std::make_shared<Frame>(RGBData{ std::move(image) }, description));
	}
	void AddDepth(std::vector<uint16_t>&& data, int w, int h, double scale, Stream description) {
		frames.emplace_back(StreamType::Depth, std::make_shared<Frame>(DepthData{ std::move(data), w, h, scale }, description));
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
	const bool Has(StreamType type) const {
		return GetFirst(type) != nullptr;
	}
	const FrameDescription GetDescription() const {
		return this->description;
	}
};