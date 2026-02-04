#pragma once

#include <string>
#include "nlohmann/json.hpp"

namespace RGBDStream {

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
		int fps;
		int bpp;
		Intrinsics intrinsics;
	};

	struct Description {
		std::string serial;
		double depthScale;
		Stream color;
		Stream depth;
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