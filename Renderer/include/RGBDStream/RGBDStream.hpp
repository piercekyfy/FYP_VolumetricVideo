#pragma once

#include "Frameset.hpp"

#include <string>
#include <functional>
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
		StreamType type;
		int fps;
		int bpp;
		Intrinsics intrinsics;
	};

	struct Description {
		std::string serial;
		double depthScale;
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

	class RGBDStream {
	public:
		const Description& GetDescription() const {
			return this->description;
		}
		virtual std::unique_ptr<Frameset> WaitForFrames(int timeout = 1000) = 0;
	protected:
		Description description;
	};
}