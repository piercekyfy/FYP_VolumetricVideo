#pragma once

#include "RGBDStream.hpp"
#include "Frameset.hpp"

#include <string_view>
#include <memory>

namespace RGBDStream {

	struct RealsenseStreamConfiguration {
		int Width;
		int Height;
		int FPS;
	};

	class RealsenseRGBDStream : public RGBDStream {
	public:
		RealsenseRGBDStream(std::string_view serial, RealsenseStreamConfiguration configuration);
		RealsenseRGBDStream(std::string_view bagPath);
		~RealsenseRGBDStream();
		std::unique_ptr<Frameset> WaitForFrames(int timeout = 1000) override;
		void Stop();
	private:
		std::string serial{};
		struct RS_Impl;
		std::unique_ptr<RS_Impl> impl{};
	};

}