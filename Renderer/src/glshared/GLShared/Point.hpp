#pragma once

#include "glm/glm.hpp"
#include <RGBDStream/Frameset.hpp>

struct Point {
	glm::vec3 position;
	glm::vec2 texcoord;
};

class Pointcloud {
private:
	int sourceWidth{ -1 };
	int sourceHeight{ -1 };
	std::vector<glm::vec2> rays{};
	std::vector<Point> points{};

	void ComputeRays(Frame* frame) {
		auto desc = frame->GetDescription();

		if (desc.intrinsics.width == sourceWidth && desc.intrinsics.height == sourceHeight)
			return;

		sourceWidth = desc.intrinsics.width;
		sourceHeight = desc.intrinsics.height;

		rays.clear();
		rays.resize(sourceWidth * sourceHeight);

		for (int v = 0; v < sourceHeight; v++) {
			for (int u = 0; u < sourceWidth; u++) {

				rays[v * sourceWidth + u] = glm::vec2{
					float(u - desc.intrinsics.ppx) / desc.intrinsics.fx,
					float(v - desc.intrinsics.ppy) / desc.intrinsics.fy
				};
			}
		}
	}
public:
	Pointcloud() {}
	const std::vector<Point> Points() const {
		return this->points;
	}
	void Process(Frameset* frameset) {
		if (frameset == nullptr || frameset->GetFirst(StreamType::Depth) == nullptr)
			throw std::exception("Invalid frame.");

		auto depthFrame = frameset->GetFirst(StreamType::Depth);
		double depthScale = frameset->GetDescription().depthScale;
		bool hasColor = frameset->Has(StreamType::Color); // TODO: alignment if resolutions don't match

		ComputeRays(depthFrame.get());

		points.clear();

		auto depthIntr = depthFrame->GetDescription().intrinsics;

		auto depth = depthFrame->AsDepth()->GetData();

		for (int v = 0; v < sourceHeight; v++) {
			for (int u = 0; u < sourceWidth; u++) {
				int index = v * sourceWidth + u;
				float z = depth[index] * depthScale;
				if (z <= 0)
					continue;

				glm::vec2 ray = rays[index];

				points.emplace_back(
					glm::vec3{ ray.x * z, -ray.y * z, z },
					glm::vec2{ u / float(sourceWidth), v / float(sourceHeight) }
				);
			}
		}
	}
};