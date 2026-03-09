#include <iostream>

#include "glm/glm.hpp"
#include "glm/gtc/matrix_transform.hpp"
#include "glm/gtc/type_ptr.hpp"

#include "GLShared/glcommon.hpp"
#include "GLShared/Point.hpp"
#include "GLShared/RenderedPoints.hpp"
#include "GLShared/RenderTexture.hpp"

#include <RGBDStream/Frameset.hpp>
#include "RGBDStream/FileRGBDStream.hpp"
#include "RGBDStream/RealsenseRGBDStream.hpp"

#include "calibration/CharucoCalibration.hpp"

#include "opencv2/core.hpp"

#include "Eigen/Core"
#include "Eigen/Dense"
#include "opencv2/core/eigen.hpp"

glm::mat4 CVToGLM(const cv::Mat& R, const cv::Mat& T) {
	Eigen::Matrix3d eigenR;
	Eigen::Vector3d eigenT;

	cv::cv2eigen(R, eigenR);
	cv::cv2eigen(T, eigenT);

	Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
	transform.block<3, 3>(0, 0) = eigenR;
	transform.block<3, 1>(0, 3) = eigenT;

	glm::mat4 result;
	for (int col = 0; col < 4; col++)
		for (int row = 0; row < 4; row++)
			result[col][row] = (float)transform(row, col);

	return result;
}

int main() {

	std::string relPath = getPathWindows();

	RGBDStream::RealsenseRGBDStream cam0{ "239622300610", {640, 480, 15, true} };
	RGBDStream::RealsenseRGBDStream cam1{ "241122306275", {640, 480, 15, true} };

	GLFWwindow* window = initSimpleResizableViewport(600, 600);

	if (window == nullptr) {
		glfwTerminate();
		return -1;
	}

	Pointcloud pc0{};
	Pointcloud pc1{};

	RenderTexture tex0{};
	RenderTexture tex1{};

	RenderedPoints r_pc0{};
	RenderedPoints r_pc1{};

	glm::mat4 projection = glm::perspective(glm::radians(45.0f), 800.0f / 600.0f, 0.1f, 100.0f);
	glm::mat4 view = glm::mat4(1.0f);
	view = glm::lookAt({ 0.0f, 0.0f, 5.0f }, glm::vec3(0.0f), glm::vec3(0.0f, 1.0f, 0.0f));
	glm::mat4 model = glm::mat4(1.0f);

	Shader shader{ (relPath + "\\shaders\\pointcloud.vert").c_str(), (relPath + "\\shaders\\pointcloud.frag").c_str() };

	glPointSize(2.0f);

	glEnable(GL_DEPTH_TEST);

	std::unique_ptr<Frameset> fs0{};
	std::unique_ptr<Frameset> fs1{};

	double lastTime = glfwGetTime();
	double timer = 0.0;

	CharucoCalibration calib{ 5, 7, 0.030, 0.015};
	StereoCalibrationResult lastResult{ false };
	bool calibrationComplete{ false };
	int icpWaitFrames = 1000;
	glm::mat4 align;

	std::cout << "Started..." << '\n';
	while (!glfwWindowShouldClose(window)) {

		double currentTime = glfwGetTime();
		double deltaTime = currentTime - lastTime;
		timer += deltaTime;
		lastTime = currentTime;

		auto newF0 = cam0.WaitForFrames(0);
		auto newF1 = cam1.WaitForFrames(0);

		if (newF0) {
			fs0 = std::move(newF0);
		}
		if (newF1) {
			fs1 = std::move(newF1);
		}

		bool framesInSync = fs0 && fs1 &&
			std::abs(fs1->GetFirst(StreamType::IR)->Timestamp - fs0->GetFirst(StreamType::IR)->Timestamp) < 50.0;


		if (fs0 != nullptr) {
			tex0.Set(fs0->GetFirst(StreamType::Color)->AsColor()->image);
			pc0.Process(fs0.get());
			r_pc0.Update(pc0.Points());
		}

		if (fs1 != nullptr) {
			tex1.Set(fs1->GetFirst(StreamType::Color)->AsColor()->image);
			pc1.Process(fs1.get());
			r_pc1.Update(pc1.Points());
		}			

		if (framesInSync) { // calibrate
			if (calib.Feed(fs0->GetFirst(StreamType::IR).get(), fs1->GetFirst(StreamType::IR).get())) {
				std::cout << "Processed Calibration Frame, currently at: " << calib.ValidPairs() << '\n';

				if (calib.ValidPairs() == 20) {
					lastResult = calib.Calibrate(fs0->GetFirst(StreamType::IR).get()->GetDescription().intrinsics, fs1->GetFirst(StreamType::IR).get()->GetDescription().intrinsics);
					if (lastResult.Success) {
						std::cout << "Completed Calibration with an error of: " << lastResult.Error << '\n';
						calibrationComplete = true;
						std::cout << "R: " << lastResult.R << '\n';
						std::cout << "T: " << lastResult.T << '\n';
						align = glm::inverse(CVToGLM(lastResult.R, lastResult.T));
					}
				}
			}
		}
		else {
			std::cout << "unsync";
		}

		fs0 = nullptr;
		fs1 = nullptr;
		

		glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

		shader.use();

		tex0.Bind(0);

		float angle = (float)glfwGetTime();

		model = glm::rotate(glm::mat4(1.0f), angle, glm::vec3(0.0f, 1.0f, 0.0f));
		glm::mat4 mvp = projection * view * model;
		
		shader.setMatrix("mvp", GL_FALSE, glm::value_ptr(mvp));

		r_pc0.Draw();

		tex1.Bind(0);

		if (calibrationComplete) {
			glm::mat4 mvp1 = projection * view * (model * align);
			shader.setMatrix("mvp", GL_FALSE, glm::value_ptr(mvp1));
		}
		else {
			shader.setMatrix("mvp", GL_FALSE, glm::value_ptr(mvp));
		}

		r_pc1.Draw();

		glfwSwapBuffers(window);
		glfwPollEvents();
	};

	glfwTerminate();

	return 0;
}