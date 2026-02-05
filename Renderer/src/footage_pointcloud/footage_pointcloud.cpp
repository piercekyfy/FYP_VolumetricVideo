#include <iostream>
#include <fstream>
#include <vector>
#include <cstdint>
#include <string>

#include "glm/glm.hpp"
#include "glm/gtc/matrix_transform.hpp"
#include "glm/gtc/type_ptr.hpp"


#include "glcommon.hpp"
#include "RGBDStream/FileRGBDStream.hpp"

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

void GetPoints(const RGBData* rgb, const DepthData* depth, RGBDStream::Intrinsics depthIntr, std::vector<Point>& buffer) {
	for (int v = 0; v < depth->height; v++) {
		for (int u = 0; u < depth->width; u++) {
			buffer.push_back(
				Point{
					project(u, v, depth->data[v * depth->width + u] * depth->scale, depthIntr.fx, depthIntr.fy, depthIntr.ppx, depthIntr.ppy),
					glm::vec2{u / float(rgb->width()), v / float(rgb->height()) }
				});
		}
	}
}

int main() {

	std::string relPath = getPathWindows();

	RGBDStream::FileRGBDStream s{ relPath + "\\frames" };
	auto fs = s.WaitForFrames();
	auto rgbFrame = fs->GetFirst(StreamType::Color)->AsColor();
	auto dFrame = fs->GetFirst(StreamType::Depth)->AsDepth();
	auto dDescriptor = s.GetDescription().GetFirst(StreamType::Depth).value().get();
		
	std::vector<Point> points;
	points.reserve(dFrame->data.size());

	GetPoints(rgbFrame, dFrame, dDescriptor.intrinsics, points);

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

	double lastTime = glfwGetTime();
	double timer = 0.0;

	while (!glfwWindowShouldClose(window)) {
		double deltaTime = glfwGetTime() - lastTime;
		lastTime = glfwGetTime();
		timer += deltaTime;

		glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

		float angle = (float)glfwGetTime();
		
		//model = glm::rotate(glm::mat4(1.0f),
		//	angle,
		//	glm::vec3(0.0f, 1.0f, 0.0f));

		glActiveTexture(GL_TEXTURE0);
		glBindTexture(GL_TEXTURE_2D, texture0);

		shader.use();
		glm::mat4 mvp = projection * view * model;
		shader.setMatrix("mvp", GL_FALSE, glm::value_ptr(mvp));
		glBindVertexArray(VAO);
		glDrawArrays(GL_POINTS, 0, static_cast<GLsizei>(points.size()));

		glfwSwapBuffers(window);
		glfwPollEvents();

		if (timer >= 0.03) {
			timer = 0.0;

			fs = s.WaitForFrames();

			if (fs == nullptr) {
				s.Reset();
				fs = s.WaitForFrames();
			}

			auto rgbFrame = fs->GetFirst(StreamType::Color)->AsColor();
			auto dFrame = fs->GetFirst(StreamType::Depth)->AsDepth();
			image = rgbFrame->image;
			points.clear();
			GetPoints(rgbFrame, dFrame, dDescriptor.intrinsics, points);

			glBindBuffer(GL_ARRAY_BUFFER, VBO);
			glBufferSubData(GL_ARRAY_BUFFER, 0, points.size() * sizeof(Point), points.data());

			glBindTexture(GL_TEXTURE_2D, texture0);
			glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, image.cols, image.rows, GL_RGB, GL_UNSIGNED_BYTE, image.data);
			glGenerateMipmap(GL_TEXTURE_2D);
		}
	};

	glDeleteVertexArrays(1, &VAO);
	glDeleteBuffers(1, &VBO);

	glfwTerminate();

	return 0;
}