#include <iostream>

#include "glm/glm.hpp"
#include "glm/gtc/matrix_transform.hpp"
#include "glm/gtc/type_ptr.hpp"

#include "glcommon.hpp"
#include "glshared/Point.hpp"
#include "glshared/RenderedPoints.hpp"
#include "glshared/RenderTexture.hpp"


#include <RGBDStream/Frameset.hpp>
#include "RGBDStream/FileRGBDStream.hpp"
#include "RGBDStream/RealsenseRGBDStream.hpp"
#include "calibration/StereoCalibrator.hpp"

#include "opencv2/core.hpp"


StereoPoseResult EstimateStereoPose(RGBDStream::RGBDStream& streamR, RGBDStream::RGBDStream& streamL, StereoCalibrator& calibrator) {
	auto fsR = streamR.WaitForFrames();
	auto fsL = streamL.WaitForFrames();

	Intrinsics intrR = fsR->GetFirst(StreamType::Color)->GetDescription().intrinsics;
	Intrinsics intrL = fsL->GetFirst(StreamType::Color)->GetDescription().intrinsics;

	while (fsR != nullptr && fsL != nullptr) {
		calibrator.Process(fsR->GetFirst(StreamType::Color).get(), fsL->GetFirst(StreamType::Color).get());

		fsR = streamR.WaitForFrames();
		fsL = streamL.WaitForFrames();
	}

	return calibrator.GetPose(intrR, intrL);
}

#include "librealsense2/rs.hpp"

glm::mat4 CVToGLM(const cv::Mat& R, const cv::Mat& T) {
	glm::mat4 transform(1.0f);

	for (int row = 0; row < 3; row++)
		for (int col = 0; col < 3; col++)
			transform[col][row] = (float)R.at<double>(row, col);

	transform[3][0] = (float)T.at<double>(0);
	transform[3][1] = (float)T.at<double>(1);
	transform[3][2] = (float)T.at<double>(2);

	return transform;
}

int main() {

	std::string relPath = getPathWindows();

	//RGBDStream::FileRGBDStream cam0{ relPath + "\\frames\\239622300610" };
	//RGBDStream::FileRGBDStream cam1{ relPath + "\\frames\\241122306275" };


	RGBDStream::RealsenseRGBDStream cam0{ "239622300610", {640, 480, 15} };
	//RGBDStream::RealsenseRGBDStream cam1{ "241122306275", {640, 480, 15} };
	auto rgbDescription = cam0.GetDescription().GetFirst(StreamType::Color).value().get();

	//StereoCalibrator calibrator{ 9, 6, 0.0253, rgbDescription.intrinsics.width, rgbDescription.intrinsics.height };

	//auto calibrationResult = EstimateStereoPose(cam0, cam1, calibrator);

	//cam0.Reset();
	//cam1.Reset();

	//if (!calibrationResult.Success) {
	//	std::cerr << "Failed to calibrate any samples. Is there a clearly visible checkerboard pattern in all cameras view?" << '\n';
	//	return -1;
	//}

	//std::cout << "Calibrated with " << calibrationResult.SuccessfulSamples << " samples and an error of: " << calibrationResult.Error << " pixels." << '\n';

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

	//glm::mat4 alignTransform = CVToGLM(calibrationResult.R, calibrationResult.T);

	while (!glfwWindowShouldClose(window)) {

		double currentTime = glfwGetTime();
		double deltaTime = currentTime - lastTime;
		timer += deltaTime;
		lastTime = currentTime;

		auto newFrame = cam0.WaitForFrames(0); // 0ms timeout, non-blocking
		if (newFrame) {
			fs0 = std::move(newFrame);
		}

		//if (fs0 == nullptr /*|| fs1 == nullptr*/) {
		//	fs0 = nullptr;
		//	fs1 = nullptr;

		//	auto new0 = cam0.WaitForFrames(99999);
		//	//auto new1 = cam1.WaitForFrames(99999);

		//	fs0 = std::move(new0);
		//	//if (new0 != nullptr && new1 != nullptr) {
		//	//	
		//	//	fs1 = std::move(new1);
		//	//}

		//	//if (fs0 == nullptr || fs1 == nullptr) {
		//	//	cam0.Reset();
		//	//	cam1.Reset();

		//	//	fs0 = cam0.WaitForFrames();
		//	//	fs1 = cam1.WaitForFrames();
		//	//}
		//}

		if (timer >= 0.3 && fs0 != nullptr /*&& fs1 != nullptr*/) { // Next Frame

			timer = 0.0;

			tex0.Set(fs0->GetFirst(StreamType::Color)->AsColor()->image);
			//tex1.Set(fs1->GetFirst(StreamType::Color)->AsColor()->image);

			pc0.Process(fs0.get());
			//pc1.Process(fs1.get());

			r_pc0.Update(pc0.Points());
			//r_pc1.Update(pc1.Points());

			fs0 = nullptr;
			fs1 = nullptr;
		}

		glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

		shader.use();

		tex0.Bind(0);

		float angle = (float)glfwGetTime();

		model = glm::rotate(glm::mat4(1.0f), angle, glm::vec3(0.0f, 1.0f, 0.0f));
		glm::mat4 mvp = projection * view * model;
		
		shader.setMatrix("mvp", GL_FALSE, glm::value_ptr(mvp));

		r_pc0.Draw();

		//tex1.Bind(0);

		//shader.setMatrix("mvp", GL_FALSE, glm::value_ptr(mvp /** glm::inverse(alignTransform)*/));

		//r_pc1.Draw();

		glfwSwapBuffers(window);
		glfwPollEvents();
	};

	glfwTerminate();

	return 0;
}