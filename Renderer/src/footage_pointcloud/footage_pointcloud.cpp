#include <iostream>
#include <fstream>
#include <vector>
#include <cstdint>
#include <string>

#include "glm/glm.hpp"
#include "glm/gtc/matrix_transform.hpp"
#include "glm/gtc/type_ptr.hpp"

#include "opencv2/opencv.hpp"

#include "glcommon.hpp"

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
	if (d <= 0)
		return glm::vec3{ 0,0,0 };

	float z = d;
	float x = (u - ppx) * (1.0f / fx) * z;
	float y = (v - ppy) * (1.0f / fy) * z;
	return glm::vec3{ x, -y, z }; // Flip y
}

struct Point {
	glm::vec3 position;
	glm::vec2 texcoord;
};

#include "RGBDStream/FileRGBDStream.hpp"

int main() {

	std::string relPath = getPathWindows();

	RGBDStream::FileRGBDStream s{ relPath + "\\frames" };
	auto fs = s.WaitForFrames();
	auto rgbFrame = fs->GetFirst(StreamType::Color)->AsColor();
	auto dFrame = fs->GetFirst(StreamType::Depth)->AsDepth();
	auto dDescriptor = s.GetDescription().GetFirst(StreamType::Depth).value().get();
		
	std::vector<Point> points;
	points.reserve(dFrame->data.size());

	for (int v = 0; v < dFrame->height; v++) {
		for (int u = 0; u < dFrame->width; u++) {
			points.push_back(
				Point{
					project(u, v, dFrame->data[v * dFrame->width + u] * dFrame->scale, dDescriptor.intrinsics.fx, dDescriptor.intrinsics.fy, dDescriptor.intrinsics.ppx, dDescriptor.intrinsics.ppy),
					glm::vec2{u / float(rgbFrame->width()), v / float(rgbFrame->height()) }
				});
		}
	}

	// Setup OpenGL

	GLFWwindow* window = initSimpleResizableViewport(600, 600);

	if (window == nullptr) {
		glfwTerminate();
		return -1;
	}

	// Get Texture

	cv::Mat image{ rgbFrame->image };

	if (image.empty())
		throw std::runtime_error("Color frame not found.");

	glActiveTexture(GL_TEXTURE0);

	GLuint texture0;
	glGenTextures(1, &texture0);
	glBindTexture(GL_TEXTURE_2D, texture0);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

	glTexImage2D(
		GL_TEXTURE_2D,
		0,
		GL_RGB,
		image.cols,
		image.rows,
		0,
		GL_RGB,
		GL_UNSIGNED_BYTE,
		image.data
	);
	glGenerateMipmap(GL_TEXTURE_2D);
	

	// Point-cloud Model-View-Projection Matrix

	// Arg1: FOV angle, Arg2: Width/Height, Arg3: zNear, Arg4: zFar
	glm::mat4 projection = glm::perspective(glm::radians(45.0f), 800.0f / 600.0f, 0.1f, 100.0f);

	glm::mat4 view = glm::mat4(1.0f);
	//// Move view back 3.0f
	view = glm::translate(view, glm::vec3(0.0f, 0.0f, -3.0f));

	view = glm::lookAt({0.0f, 0.0f, 5.0f}, glm::vec3(0.0f), glm::vec3(0.0f, 1.0f, 0.0f));

	glm::mat4 model = glm::mat4(1.0f);

	// Setup Shader
	Shader shader{ (relPath + "\\shaders\\pointcloud.vert").c_str(), (relPath + "\\shaders\\pointcloud.frag").c_str() };

	GLuint VAO, VBO;
	glGenVertexArrays(1, &VAO);
	glGenBuffers(1, &VBO);

	glBindVertexArray(VAO);

	glBindBuffer(GL_ARRAY_BUFFER, VBO);
	glBufferData(GL_ARRAY_BUFFER, points.size() * sizeof(Point), points.data(), GL_STATIC_DRAW);

	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Point), (void*)0);
	glEnableVertexAttribArray(0);
	glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, sizeof(Point), (void*)offsetof(Point, texcoord));
	glEnableVertexAttribArray(1);

	glPointSize(2.0f);

	glEnable(GL_DEPTH_TEST);

	while (!glfwWindowShouldClose(window)) {
		glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);


		float angle = (float)glfwGetTime();
		
		model = glm::rotate(glm::mat4(1.0f),
			angle,
			glm::vec3(0.0f, 1.0f, 0.0f));

		glActiveTexture(GL_TEXTURE0);
		glBindTexture(GL_TEXTURE_2D, texture0);

		shader.use();
		glm::mat4 mvp = projection * view * model;
		shader.setMatrix("mvp", GL_FALSE, glm::value_ptr(mvp));
		glBindVertexArray(VAO);
		glDrawArrays(GL_POINTS, 0, static_cast<GLsizei>(points.size()));

		glfwSwapBuffers(window);
		glfwPollEvents();
	};

	glDeleteVertexArrays(1, &VAO);
	glDeleteBuffers(1, &VBO);

	glfwTerminate();

	return 0;
}