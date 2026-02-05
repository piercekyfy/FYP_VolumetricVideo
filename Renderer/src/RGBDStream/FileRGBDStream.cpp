#include "RGBDStream/FileRGBDStream.hpp"
#include "RGBDStream/Frameset.hpp"

#include <iostream>
#include <sstream>
#include <iomanip>

static bool DirExists(std::string_view path) {
	return std::filesystem::exists(path) && std::filesystem::is_directory(path);
}

namespace RGBDStream {

	static void from_json(const nlohmann::json& j, Intrinsics& intr) {
		j.at("width").get_to(intr.width);
		j.at("height").get_to(intr.height);
		j.at("ppx").get_to(intr.ppx);
		j.at("ppy").get_to(intr.ppy);
		j.at("fx").get_to(intr.fx);
		j.at("fy").get_to(intr.fy);

		for (int i = 0; i < 5; i++) {
			j.at("coeffs").at(i).get_to(intr.coeffs[i]);
		}
	}

	static void from_json(const nlohmann::json& j, Stream& s) {
		j.at("stream_type").get_to(s.type);
		j.at("fps").get_to(s.fps);
		j.at("bpp").get_to(s.bpp);
		j.at("intrinsics").get_to(s.intrinsics);
	}

	static void from_json(const nlohmann::json& j, Description& d) {
		j.at("serial").get_to(d.serial);
		j.at("depth_scale").get_to(d.depthScale);

		for (const auto& js : j.at("streams")) {
			d.streams.push_back(js.get<Stream>());
		}
	}

	FileRGBDStream::FileRGBDStream(std::string_view sourcePath) : sourcePath{ std::filesystem::path{sourcePath} } {

		if (!DirExists(sourcePath))
			throw std::runtime_error{ "Directory does not exist: " + this->sourcePath.string() };

		std::ifstream descriptionFile{ this->sourcePath / "description.json" };

		if (!descriptionFile)
			throw std::runtime_error("Description file not found at: " + (this->sourcePath / "description.json").string());

		this->description = nlohmann::json::parse(descriptionFile).get<Description>();
	}

	std::unique_ptr<Frameset> FileRGBDStream::WaitForFrames(int timeout) {
		auto fs = std::make_unique<Frameset>();

		for (const auto& stream : this->description.streams) {
			std::string prefix = stream.type == StreamType::Depth ? ".bin" : ".png";
			std::string directory = std::to_string(static_cast<int>(stream.type));
			std::string fileName = (std::ostringstream{} << std::setfill('0') << std::setw(6) << this->fileIndex).str() + prefix;
			std::string filePath = (this->sourcePath / directory / fileName).string();

			switch (stream.type) {
			case StreamType::Color: { // Dividing by the size of a uint8 is redundant.

				cv::Mat image = cv::imread(filePath, cv::IMREAD_COLOR); // This sends out a log when it fails. TODO: remember to disable.
				
				if (image.empty())
					continue;

				cv::cvtColor(image, image, cv::COLOR_BGR2RGB);

				(*fs).AddColor(std::move(image));
				break;
			}
			case StreamType::Depth: {

				size_t expectedSize = stream.intrinsics.width * stream.intrinsics.height * stream.bpp;

				std::vector<uint16_t> buffer(expectedSize / sizeof(uint16_t));

				this->fileReader.open(filePath, std::ios::binary);

				if (!this->fileReader)
					continue;

				fileReader.read(reinterpret_cast<char*>(buffer.data()), expectedSize);

				(*fs).AddDepth(std::move(buffer), stream.intrinsics.width, stream.intrinsics.height, this->description.depthScale);

				this->fileReader.close();
				break;
			}
			}
		}

		if ((*fs).Size() <= 0)
			return nullptr;

		++fileIndex;

		return fs;
	}
}