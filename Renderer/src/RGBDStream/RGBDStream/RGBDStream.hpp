#pragma once

#include "Frameset.hpp"

#include <string>
#include <functional>
#include <stdexcept>
#include <concepts>

#include "nlohmann/json.hpp"

namespace RGBDStream {
	
	class RGBDStream {
	protected:
		FrameDescription description;
	public:
		const FrameDescription& GetDescription() const {
			return this->description;
		}
		virtual std::unique_ptr<Frameset> WaitForFrames(int timeout = 1000) = 0;
	};
}