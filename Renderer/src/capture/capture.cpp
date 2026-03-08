#include <iostream>
#include "librealsense2/rs.hpp"

int main() {
	std::cout << "Hello world";
	rs2::context ctx{};
	
	rs2::device_list devices = ctx.query_devices();

	std::cout << "Found " << devices.size() << " RealSense device(s).\n";
}