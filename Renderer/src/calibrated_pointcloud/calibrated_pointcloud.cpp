#include <iostream>

#include "glm/glm.hpp"
#include "glm/gtc/matrix_transform.hpp"
#include "glm/gtc/type_ptr.hpp"

#include "Eigen/Core"
#include "Eigen/Dense"
#include "opencv2/core.hpp"
#include "opencv2/core/eigen.hpp"

#include "GLShared/glcommon.hpp"
#include "GLShared/Point.hpp"
#include "GLShared/RenderedPoints.hpp"
#include "GLShared/RenderTexture.hpp"

#include <RGBDStream/Frameset.hpp>
#include "RGBDStream/RealsenseStream.hpp"
#include "Calibration/CharucoCalibration.hpp"

Eigen::Matrix4d CVToEigen(const cv::Mat& R, const cv::Mat& T) {
	Eigen::Matrix3d eigenR;
	Eigen::Vector3d eigenT;

	cv::cv2eigen(R, eigenR);
	cv::cv2eigen(T, eigenT);

	Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
	transform.block<3, 3>(0, 0) = eigenR;
	transform.block<3, 1>(0, 3) = eigenT;

	return transform;
}

glm::mat4 CVToGLM(const cv::Mat& R, const cv::Mat& T) {
	auto transform = CVToEigen(R, T);

	glm::mat4 result;
	for (int col = 0; col < 4; col++)
		for (int row = 0; row < 4; row++)
			result[col][row] = (float)transform(row, col); // transpose

	return result;
}

glm::mat4 EigenToGlm(const Eigen::Matrix4d& v)
{
	glm::mat4 result;
	for (size_t i = 0; i < 4; ++i) {
		for (size_t j = 0; j < 4; ++j) {
			result[i][j] = v(j, i);
		}
	}

	return result;
}

Eigen::Isometry3d GLMToEigen(const glm::mat4& m) {
	Eigen::Matrix4d mat;
	for (int col = 0; col < 4; col++)
		for (int row = 0; row < 4; row++)
			mat(row, col) = (double)m[col][row];
	return Eigen::Isometry3d(mat);
}


class CameraState {
public:
	const glm::vec3 WorldUp = { 0,1,0 };

	CameraState(glm::vec3 startPosition, glm::vec3 up, float yaw, float pitch) : position(startPosition), up(up), forward({ 0,0,-1 }), right({ 1,0,0 }), yaw(yaw), pitch(pitch)
	{
		calculateVectors();
	}

	glm::mat4 PerspectiveMatrix() {
		return glm::perspective(glm::radians(45.0f), 800.0f / 600.0f, 0.1f, 100.0f);
	}

	glm::mat4 ViewMatrix() {
		return glm::lookAt(position, position + forward, up);
	}

	void ProcessInput(int key, float deltaTime) {
		float velocity = movementSpeed * deltaTime;
		float rotationVelocity = rotationSpeed * deltaTime;

		if (key == GLFW_KEY_W) position += forward * velocity;
		if (key == GLFW_KEY_S) position -= forward * velocity;
		if (key == GLFW_KEY_A) position -= right * velocity;
		if (key == GLFW_KEY_D) position += right * velocity;

		if (key == GLFW_KEY_LEFT)  yaw -= rotationSpeed;
		if (key == GLFW_KEY_RIGHT) yaw += rotationSpeed;
		if (key == GLFW_KEY_UP)    pitch += rotationSpeed;
		if (key == GLFW_KEY_DOWN)  pitch -= rotationSpeed;

		if (pitch > 89.0f) pitch = 89.0f;
		if (pitch < -89.0f) pitch = -89.0f;

		calculateVectors();
	}
private:
	glm::vec3 position;
	glm::vec3 forward;
	glm::vec3 up;
	glm::vec3 right;

	float yaw;
	float pitch;

	float movementSpeed{2.5f};
	float rotationSpeed{ 2.0f };

	void calculateVectors() {
		glm::vec3 forward;

		// angle into direction vector. x = cos yaw * cos pitch, y = sin pitch, z = sin yaw * cos pitch
		forward.x = cos(glm::radians(yaw)) * cos(glm::radians(pitch));
		forward.y = sin(glm::radians(pitch));
		forward.z = sin(glm::radians(yaw)) * cos(glm::radians(pitch));
		this->forward = glm::normalize(forward);
		// orthogonal
		right = glm::normalize(glm::cross(forward, WorldUp));
		up = glm::normalize(glm::cross(right, forward));
	}
};


int main() {

	std::string relPath = getPathWindows();

	RGBDStream::RealsenseStreamConfiguration calibrationConfig{};
	calibrationConfig.UseBag = true;
	calibrationConfig.EnableIR = true;

	RGBDStream::RealsenseStreamConfiguration captureConfig{};
	captureConfig.UseBag = true;

	std::vector<std::string> calibrationSources{
		"C:\\Users\\pierc\\Downloads\\vv_footage\\2cam_calib_239622300610.bag",
		"C:\\Users\\pierc\\Downloads\\vv_footage\\2cam_calib_241122306275.bag"
	};

	std::vector<std::string> captureSources{
		"C:\\Users\\pierc\\Downloads\\vv_footage\\2cam_still_239622300610.bag",
		"C:\\Users\\pierc\\Downloads\\vv_footage\\2cam_still_241122306275.bag"
	};

	std::unique_ptr<RGBDStream::RGBDStreamGroup<RGBDStream::RealsenseStream>> calibrationCapture;

	if (calibrationConfig.UseBag) {
		calibrationCapture = std::make_unique<RGBDStream::RealsensePlaybackStreamGroup>(calibrationSources, std::vector<RGBDStream::RealsenseStreamConfiguration>{ calibrationConfig });
	}
	else {
		calibrationCapture = std::make_unique<RGBDStream::RealsenseStreamGroup>(calibrationSources, std::vector<RGBDStream::RealsenseStreamConfiguration>{ calibrationConfig });
	}

	// Perform Calibration

	CharucoCalibration calib{ 5, 7, 0.030, 0.015 };
	StereoCalibrationResult calibrationResult{ false };

	while (!calibrationResult.Success) {
		auto framesets = calibrationCapture->WaitForSynchronizedFrames();
		Frameset* source = framesets[0].get();
		Frameset* target = framesets[1].get();

		if (source == nullptr || target == nullptr) {
			std::cout << "Could not complete calibration before stream ended.\n";
			return -1;
		}

		Frame* sourceFrame;
		Frame* targetFrame;

		if (calibrationConfig.EnableIR) {
			sourceFrame = source->GetFirst(StreamType::IR).get();
			targetFrame = target->GetFirst(StreamType::IR).get();
		}
		else {
			sourceFrame = source->GetFirst(StreamType::Color).get();
			targetFrame = target->GetFirst(StreamType::Color).get();
		}

		calib.Feed(sourceFrame, targetFrame);

		if (calib.ValidPairs() >= 20 && calib.ValidPairs() % 20 == 0) {
			calibrationResult = calib.Calibrate(sourceFrame->GetDescription().intrinsics, targetFrame->GetDescription().intrinsics);
			std::cout << "Calibration " << (calibrationResult.Success ? "success" : "failure") << " with error of: " << std::to_string(calibrationResult.Error) << '\n';
		}
	}

	glm::mat4 sourceToTargetMat = CVToGLM(calibrationResult.R, calibrationResult.T);

	// Setup Camera

	CameraState cameraState{ {0,0,-3}, {0,1,0}, 90, 0 };

	std::vector<int> inputKeys = { GLFW_KEY_W, GLFW_KEY_A, GLFW_KEY_D, GLFW_KEY_S, GLFW_KEY_UP, GLFW_KEY_LEFT, GLFW_KEY_RIGHT, GLFW_KEY_DOWN };

	// Setup Graphics Pipeline

	GLFWwindow* window = initSimpleResizableViewport(600, 600);

	if (window == nullptr) {
		glfwTerminate();
		return -1;
	}

	Pointcloud pc0{};
	Pointcloud pc1{};

	// Refine calibration with ICP

	std::unique_ptr<RGBDStream::RGBDStreamGroup<RGBDStream::RealsenseStream>> calib_capture;

	if (captureConfig.UseBag) {
		calib_capture = std::make_unique<RGBDStream::RealsensePlaybackStreamGroup>(captureSources, std::vector<RGBDStream::RealsenseStreamConfiguration>{ captureConfig });
	}
	else {
		calib_capture = std::make_unique<RGBDStream::RealsenseStreamGroup>(captureSources, std::vector<RGBDStream::RealsenseStreamConfiguration>{ captureConfig });
	}

	auto fss = calib_capture->WaitForSynchronizedFrames();

	pc0.Process(fss[0].get());
	pc1.Process(fss[1].get());

	ICPCalibrationResult icpResult = icp(pc0, pc1, GLMToEigen(sourceToTargetMat), 0.05, 0.15);
	sourceToTargetMat = EigenToGlm(icpResult.Transform.matrix());

	std::cout << "Calibration Refinement " << (icpResult.Success ? "success" : "failure") << " with error of: " << std::to_string(icpResult.Error) << '\n';

	pc0.Translation = sourceToTargetMat;
	
	calib_capture = nullptr;

	RenderTexture tex0{};
	RenderTexture tex1{};

	tex0.Set(fss[0].get()->GetFirst(StreamType::Color)->AsColor()->image);
	tex1.Set(fss[1].get()->GetFirst(StreamType::Color)->AsColor()->image);

	RenderedPoints r_pc0{};
	RenderedPoints r_pc1{};

	Shader shader{ (relPath + "\\shaders\\pointcloud.vert").c_str(), (relPath + "\\shaders\\pointcloud.frag").c_str() };

	glPointSize(2.0f);

	glEnable(GL_DEPTH_TEST);

	// Initialize Capture

	std::unique_ptr<RGBDStream::RGBDStreamGroup<RGBDStream::RealsenseStream>> capture;

	if (captureConfig.UseBag) {
		capture = std::make_unique<RGBDStream::RealsensePlaybackStreamGroup>(captureSources, std::vector<RGBDStream::RealsenseStreamConfiguration>{ captureConfig });
	}
	else {
		capture = std::make_unique<RGBDStream::RealsenseStreamGroup>(captureSources, std::vector<RGBDStream::RealsenseStreamConfiguration>{ captureConfig });
	}

	// Render Loop

	const double FPS = 15.00;
	const double FRAME = 1.0 / FPS;

	std::unique_ptr<Frameset> fsSource;
	std::unique_ptr<Frameset> fsTarget;

	double lastTime = glfwGetTime();
	double accumDelta = 0.0;

	r_pc0.Update(pc0.Points());
	r_pc1.Update(pc1.Points());
	while (!glfwWindowShouldClose(window)) {
		
		double currentTime = glfwGetTime();
		double deltaTime = currentTime - lastTime;
		lastTime = currentTime;
		accumDelta += deltaTime;

		for (auto key : inputKeys) {
			if (glfwGetKey(window, key)) {
				cameraState.ProcessInput(key, deltaTime);
			}
		}

		if (accumDelta >= FRAME) {

			auto fss = capture->WaitForSynchronizedFrames(0);

			if (fss[0] != nullptr) {
				fsSource = std::move(fss[0]);
			}
			if (fss[1] != nullptr) {
				fsTarget = std::move(fss[1]);
			}

			accumDelta = 0;
		}

		if (fsSource != nullptr) {
			tex0.Set(fsSource->GetFirst(StreamType::Color)->AsColor()->image);
			pc0.Process(fsSource.get());
			r_pc0.Update(pc0.Points());
		}

		if (fsTarget != nullptr) {
			tex1.Set(fsTarget->GetFirst(StreamType::Color)->AsColor()->image);
			pc1.Process(fsTarget.get());
			r_pc1.Update(pc1.Points());
		}

		glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

		shader.use();

		tex0.Bind(0);

		shader.setMatrix("mvp", GL_FALSE, glm::value_ptr(cameraState.PerspectiveMatrix() * cameraState.ViewMatrix() * pc0.Translation));

		r_pc0.Draw();

		tex1.Bind(0);

		shader.setMatrix("mvp", GL_FALSE, glm::value_ptr(cameraState.PerspectiveMatrix() * cameraState.ViewMatrix() * pc1.Translation));

		r_pc1.Draw();

		glfwSwapBuffers(window);
		glfwPollEvents();
	};

	glfwTerminate();

	return 0;
}