#pragma once

#include "Frameset.hpp"
#include "RGBDStream.hpp"

#include <string>
#include <functional>
#include <stdexcept>
#include <concepts>

#include "nlohmann/json.hpp"

namespace RGBDStream {
	template<std::derived_from<RGBDStream> T>
	class RGBDStreamGroup {
	protected:
		std::vector<FrameDescription> descriptions;
	public:
		virtual const FrameDescription& GetDescription(int index) const = 0;
		virtual const T& GetStream(int index) const = 0;
		/// <summary>
		/// Attempts to return a frameset from all streams.
		/// </summary>
		/// <returns>A vector of framesets, or nullptr if capture failed.</returns>
		virtual std::vector<std::unique_ptr<Frameset>> WaitForFrames(int timeout = 1000) = 0;
		/// <summary>
		/// Attempts to return synchronized framesets from all streams. This method may skip any number of frames from any streams to accomplish this.
		/// </summary>
		/// <returns>A vector of framesets, or of nullptrs if it was not possible to find a synchronized set.</returns>
		virtual std::vector<std::unique_ptr<Frameset>> WaitForSynchronizedFrames(int timeout = 1000, double maxDelta = 50) = 0;
	};
}