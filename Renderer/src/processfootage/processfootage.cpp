#include <iostream>
#include <fstream>
#include <vector>
#include <cstdint>
#include <string>

#include "glm/glm.hpp"
#include "nlohmann/json.hpp"

struct intrinsics {
	int width;
	int height;
	int ppx;
	int ppy;
	int fx;
	int fy;
	int coeffs[5];
};

struct stream {
	int fps;
	int bpp;
	intrinsics intrinsics;
};

struct description {
	std::string serial;
	double depthScale;
	stream color;
	stream depth;
};


void from_json(const nlohmann::json& j, intrinsics& intr) {
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

void from_json(const nlohmann::json& j, stream& s) {
	j.at("fps").get_to(s.fps);
	j.at("bpp").get_to(s.bpp);
	j.at("intrinsics").get_to(s.intrinsics);
}

void from_json(const nlohmann::json& j, description& d) {
	j.at("serial").get_to(d.serial);
	j.at("depth_scale").get_to(d.depthScale);
	j.at("streams").at("color").get_to(d.color);
	j.at("streams").at("depth").get_to(d.depth);
}

glm::vec3 project(int u, int v, float d, float fx, float fy, float ppx, float ppy) {
	float z = d;
	float x = (u - ppx) * (1.0f / fx) * z;
	float y = (v - ppy) * (1.0f / fy) * z;
	return glm::vec3{ x,y,z };
}

int main() {

	std::ifstream description_file("C:\\Users\\pierc\\Downloads\\SingleCamForward\\241122306275\\description.json");
	if (!description_file)
		return 1;

	nlohmann::json description_json = nlohmann::json::parse(description_file);
	description d = description_json.get<description>();

	int stride = d.depth.intrinsics.width * d.depth.bpp;
	int file_bytes_size = stride * d.depth.intrinsics.height;

	std::ifstream input("C:\\Users\\pierc\\Downloads\\SingleCamForward\\241122306275\\depth\\000000.bin", std::ios::binary);

	std::vector<uint16_t> data(d.depth.intrinsics.width * d.depth.intrinsics.height);

	input.read(reinterpret_cast<char*>(data.data()), file_bytes_size);
	input.close();

	std::vector<glm::vec3> points;
	points.reserve(data.size());

	for (int u = 0; u < d.depth.intrinsics.width; u++) {
		for (int v = 0; v < d.depth.intrinsics.height; v++) {
			points.push_back(project(u, v, data[v * d.depth.intrinsics.width + u] * d.depthScale, d.depth.intrinsics.fx, d.depth.intrinsics.fy, d.depth.intrinsics.ppx, d.depth.intrinsics.ppy));
		}
	}

	return 0;
}