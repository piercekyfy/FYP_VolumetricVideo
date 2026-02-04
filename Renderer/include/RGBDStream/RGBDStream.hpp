#pragma once

#include <string>
#include "nlohmann/json.hpp"

namespace RGBDStream {

	enum StreamType {
		Color = 0,
		Depth = 1
	}; // I want to keep this implementation open to the addition alternative streams like IR.

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

	struct Description {
		std::string serial;
		double depthScale;
		std::vector<Stream> streams{};
	};

	class RGBDStream {
	public:
		const Description& GetDescription() const {
			return this->description;
		}
	protected:
		Description description;
	};
}