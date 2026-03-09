#pragma once

#include "opencv2/opencv.hpp"

#include <vector>
#include <utility>
#include <variant>
#include <memory>

static const int RGB_CHANNELS = 3;

enum class StreamType : int {
	Color = 0,
	Depth = 1,
	IR = 2
};

struct RGBData { // TODO: Capitalize to follow naming convention
		// (this takes so long IDE can't refactor c++ names easily)
	cv::Mat image;
	int width() const { return image.cols; }
	int height() const { return image.rows; }
};

struct DepthData {
	int width;
	int height;
	double scale;

	DepthData(std::unique_ptr<uint16_t[]> buffer, int width, int height, double scale) : data(std::move(buffer)), width(width), height(height), scale(scale) {}
	virtual const uint16_t* GetData() const {
		return data.get();
	}
	size_t Size() const {
		return stride() * this->height;
	}
	virtual int stride() const { return width * sizeof(uint16_t); }
protected:
	DepthData() = default;
private:
	std::unique_ptr<uint16_t[]> data;
};

struct Intrinsics {
	int width;
	int height;
	float ppx;
	float ppy;
	float fx;
	float fy;
	float coeffs[5];
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
	std::variant<std::unique_ptr<RGBData>, std::unique_ptr<DepthData>> data;
	const double Timestamp;

	Frame(std::unique_ptr<RGBData> data, Stream description, double timestamp) : data(std::move(data)), description(description), Timestamp(timestamp) {}
	Frame(std::unique_ptr<DepthData> data, Stream description, double timestamp) : data(std::move(data)), description(description), Timestamp(timestamp) {}

	const RGBData* AsColor() const {
		auto ptr = std::get_if<std::unique_ptr<RGBData>>(&data);
		return ptr ? ptr->get() : nullptr;
	}
	const DepthData* AsDepth() const {
		auto ptr = std::get_if<std::unique_ptr<DepthData>>(&data);
		return ptr ? ptr->get() : nullptr;
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
	void AddColor(std::unique_ptr<RGBData> data, Stream description, double timestamp = 0) {
		frames.emplace_back(StreamType::Color, std::make_shared<Frame>(std::move(data), description, timestamp));
	}
	void AddIR(std::unique_ptr<RGBData> data, Stream description, double timestamp = 0) {
		frames.emplace_back(StreamType::IR, std::make_shared<Frame>(std::move(data), description, timestamp));
	}
	void AddDepth(std::unique_ptr<DepthData> data, Stream description, double timestamp = 0) {
		frames.emplace_back(StreamType::Depth, std::make_shared<Frame>(std::move(data), description, timestamp));
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