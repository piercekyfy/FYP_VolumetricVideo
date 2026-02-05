#pragma once

#include "RGBDStream.hpp"
#include "Frameset.hpp"

#include <string_view>
#include <fstream>
#include <filesystem>

namespace RGBDStream {
	class FileRGBDStream : public RGBDStream {
	public:
		FileRGBDStream(std::string_view sourcePath);
		std::unique_ptr<Frameset> WaitForFrames(int timeout = 1000) override;
		void Reset() {
			fileReader.close();
			fileIndex = 0;
		}
	private:
		std::filesystem::path sourcePath;
		std::ifstream fileReader{};
		size_t fileIndex{ 0 };
	};
}