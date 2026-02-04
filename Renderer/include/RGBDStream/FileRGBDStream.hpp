#pragma once

#include "RGBDStream.hpp"

#include <string_view>
#include <filesystem>

namespace RGBDStream {
	class FileRGBDStream : public RGBDStream {
	public:
		FileRGBDStream(std::string_view sourcePath);
	private:
		std::filesystem::path sourcePath;
	};
}