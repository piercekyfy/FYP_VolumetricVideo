#include "RGBDStream/FileRGBDStream.hpp"

#include <iostream>

#include <fstream>
#include "opencv2/opencv.hpp"

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
		j.at("fps").get_to(s.fps);
		j.at("bpp").get_to(s.bpp);
		j.at("intrinsics").get_to(s.intrinsics);
	}

	static void from_json(const nlohmann::json& j, Description& d) {
		j.at("serial").get_to(d.serial);
		j.at("depth_scale").get_to(d.depthScale);
		j.at("streams").at("color").get_to(d.color);
		j.at("streams").at("depth").get_to(d.depth);
	}

	FileRGBDStream::FileRGBDStream(std::string_view sourcePath) : sourcePath{ std::filesystem::path{sourcePath} } {

		if (!DirExists(sourcePath))
			throw std::runtime_error{ "Directory does not exist: " + this->sourcePath.string() };

		std::ifstream descriptionFile{ this->sourcePath / "description.json" };

		if (!descriptionFile)
			throw std::runtime_error("Description file not found at: " + (this->sourcePath / "description.json").string());
		
		this->description = nlohmann::json::parse(descriptionFile).get<Description>();

		std::cout << this->description.depthScale;
	}

}