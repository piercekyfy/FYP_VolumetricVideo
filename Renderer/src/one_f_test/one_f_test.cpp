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

#include "Reconstruction/SparseVoxelGrid.hpp"
#include "Reconstruction/MarchingCubes.hpp"
#include "Reconstruction/RenderedMesh.hpp"

#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"

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

	float movementSpeed{ 2.5f };
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

struct AppState {
	bool UseBag = false;
	char source0BagCalibPath[512] = "C:\\Users\\pierc\\Downloads\\vv_footage\\2cam_calib_239622300610.bag";
	char source1BagCalibPath[512] = "C:\\Users\\pierc\\Downloads\\vv_footage\\2cam_calib_241122306275.bag";
	char source0BagPath[512] = "C:\\Users\\pierc\\Downloads\\vv_footage\\2cam_still_239622300610.bag";
	char source1BagPath[512] = "C:\\Users\\pierc\\Downloads\\vv_footage\\2cam_still_241122306275.bag";
	char source0Serial[512] = "";
	char source1Serial[512] = "";
	bool ReloadRequested = false;

	float TSDFTruncation = 0.02f;
	float VoxelSize = 0.01f;

	bool CalibrationRequested = false;
	bool ICPCalibrationRequested = false;
	bool Calibrated = false;
	bool ICPRefined = false;
	float LastCalibrationError = 0.0f;
	float LastICPCalibrationError = 0.0f;
	glm::mat4 CalibratedTransform{ 1.0f };

	bool Render = false;
	enum class RenderMode { MESH, PC_0, PC_1, PC_BOTH } RenderMode = RenderMode::MESH;
	float PointSize = 2.0f;
	bool WireFrame = false;
	bool VoxelGrid = false;

	bool VoxelReloadRequested = false;

	bool PauseMode = true;
	bool NextRequested = false;
};

struct PCState {
	Pointcloud Source0;
	Pointcloud Source1;
	RenderTexture Texture0;
	RenderTexture Texture1;
	RenderedPoints RenderSource0;
	RenderedPoints RenderSource1;

	void Update(Frameset* fs0, glm::mat4 t0, Frameset* fs1, glm::mat4 t1) {

		if (fs0) {
			Texture0.Set(fs0->GetFirst(StreamType::Color)->AsColor()->image);
			Source0.Translation = t0;
			Source0.Process(fs0);
			RenderSource0.Update(Source0.Points());
		}

		if (fs1) {
			Texture1.Set(fs1->GetFirst(StreamType::Color)->AsColor()->image);
			Source1.Translation = t1;
			Source1.Process(fs1);
			RenderSource1.Update(Source1.Points());
		}
	}
};

struct VoxelState {
	std::unique_ptr<SparseVoxelGrid<8>> Grid;
	RenderedMesh RenderSource;

	void Update(Frameset* fs0, glm::mat4 t0, Frameset* fs1, glm::mat4 t1) {
		if (Grid == nullptr)
			return;

		if (fs0 || fs1) {
			Grid->Clear();
		}

		if (fs0) {
			Integrate(*Grid, fs0, t0);
		}

		if (fs1) {
			Integrate(*Grid, fs1, t1);
		}

		if (fs0 || fs1) {
			auto [points, indices] = MarchingCubes(*Grid);
			RenderSource.Update(points, indices);
		}
	}
};

void DrawUI(AppState& state, const CameraState& camera) {
	using namespace ImGui;
	
	ImGui_ImplOpenGL3_NewFrame();
	ImGui_ImplGlfw_NewFrame();

	NewFrame();

	ImGuiIO& io = ImGui::GetIO();

	SetNextWindowPos({ io.DisplaySize.x * 0.05f, io.DisplaySize.y * 0.05f }, ImGuiCond_Once);
	SetNextWindowSize({ io.DisplaySize.x * 0.4f, 0 }, ImGuiCond_Always);
	Begin("Source", nullptr, ImGuiWindowFlags_NoResize);
	Text("Input Source");
	SameLine();
	if (RadioButton(".bag", state.UseBag)) state.UseBag = true;
	SameLine();
	if (RadioButton("Live", !state.UseBag)) state.UseBag = false;


	BeginDisabled(state.ReloadRequested);
	if (state.ReloadRequested) {
		Separator();
		Text("Loading Sources...");
		Separator();
		
	}

	if (state.UseBag) {
		Separator();
		Text("Calibration bags");
		InputText("Source 0##cap0", state.source0BagCalibPath, sizeof(state.source0BagCalibPath));
		InputText("Source 1##cap1", state.source1BagCalibPath, sizeof(state.source1BagCalibPath));
		Spacing();
		Text("Render bags");
		InputText("Source 0##cal0", state.source0BagPath, sizeof(state.source0BagPath));
		InputText("Source 1##cal1", state.source1BagPath, sizeof(state.source1BagPath));
	}
	else {
		Separator();
		Text("Serials");
		InputText("Source 0", state.source0Serial, sizeof(state.source0Serial));
		InputText("Source 1", state.source1Serial, sizeof(state.source1Serial));
	}
	
	Spacing();
	if (Button("Reload")) state.ReloadRequested = true;

	EndDisabled();

	End();

	ImGui::SetNextWindowPos({ io.DisplaySize.x * 0.05f, io.DisplaySize.y * 0.1f }, ImGuiCond_Once);
	ImGui::SetNextWindowSize({ io.DisplaySize.x * 0.4f, 0 }, ImGuiCond_Always);
	ImGui::Begin("Calibration", nullptr, ImGuiWindowFlags_NoResize);

	Separator();

	if (state.Calibrated) {
		Text("Calibration Error: %.4f", state.LastCalibrationError);
	}
	if (state.ICPRefined) {
		Text("ICP Error: %.4f", state.LastICPCalibrationError);
	}

	Separator();

	Spacing();

	BeginDisabled(state.CalibrationRequested);
	if (Button("Run Calibration")) state.CalibrationRequested = true;
	EndDisabled();
	BeginDisabled(!state.Calibrated || state.ICPCalibrationRequested);
	if (Button("Refine with ICP")) state.ICPCalibrationRequested = true;
	EndDisabled();

	End();


	ImGui::SetNextWindowPos({ io.DisplaySize.x * 0.05f, io.DisplaySize.y * 0.15f }, ImGuiCond_Once);
	ImGui::SetNextWindowSize({ io.DisplaySize.x * 0.4f, 0 }, ImGuiCond_Always);
	ImGui::Begin("Rendering", nullptr, ImGuiWindowFlags_NoResize);

	Text("Render Mode");
	ImGui::RadioButton("Mesh", (int*)&state.RenderMode, (int)AppState::RenderMode::MESH);
	ImGui::SameLine();
	ImGui::RadioButton("Pointcloud 0", (int*)&state.RenderMode, (int)AppState::RenderMode::PC_0);
	ImGui::SameLine();
	ImGui::RadioButton("Pointcloud 1", (int*)&state.RenderMode, (int)AppState::RenderMode::PC_1);
	ImGui::SameLine();
	ImGui::RadioButton("Both PCs", (int*)&state.RenderMode, (int)AppState::RenderMode::PC_BOTH);

	ImGui::Checkbox("Wireframe", &state.WireFrame);
	ImGui::Checkbox("Voxel Grid", &state.VoxelGrid);
	ImGui::SliderFloat("Point size", &state.PointSize, 1.0f, 8.0f);
	ImGui::SliderFloat("Voxel size", &state.VoxelSize, 0.005f, 0.05f, "%.3f");

	state.TSDFTruncation = std::max(state.TSDFTruncation, state.VoxelSize * 2.0f);
	ImGui::SliderFloat("TSDF truncation", &state.TSDFTruncation, 0.005f, 0.1f, "%.3f");


	BeginDisabled(state.VoxelReloadRequested);
	if (Button("Reload Voxel Grid (Voxel/TSDF)")) state.VoxelReloadRequested = true;
	EndDisabled();

	End();

	SetNextWindowPos({ (io.DisplaySize.x / 2) - (io.DisplaySize.x * 0.15f), io.DisplaySize.y - (io.DisplaySize.y * 0.05f)}, ImGuiCond_Always);
	ImGui::SetNextWindowSize({ io.DisplaySize.x * 0.15f, 0 }, ImGuiCond_Always);
	SetNextWindowBgAlpha(0.1f);
	Begin("Controls", nullptr, ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoResize);

	if (ImGui::Button(state.PauseMode ? ">" : "||")) state.PauseMode = !state.PauseMode;
	SameLine();
	if (ImGui::Button(">|")) state.NextRequested = true;

	End();

	Render();
	ImGui_ImplOpenGL3_RenderDrawData(GetDrawData());
}

StereoCalibrationResult CharucoCalibrate(RGBDStream::RGBDStreamGroup<RGBDStream::RealsenseStream>& group, int minValids = 20, bool useIR = true) {
	CharucoCalibration calib{ 5, 7, 0.030, 0.015 };
	StereoCalibrationResult calibrationResult{ false };

	while (!calibrationResult.Success) {
		auto framesets = group.WaitForSynchronizedFrames();
		Frameset* source = framesets[0].get();
		Frameset* target = framesets[1].get();

		if (source == nullptr || target == nullptr) {
			std::cout << "Could not complete calibration before stream ended.\n";
			return { false };
		}

		Frame* sourceFrame;
		Frame* targetFrame;

		if (useIR) {
			sourceFrame = source->GetFirst(StreamType::IR).get();
			targetFrame = target->GetFirst(StreamType::IR).get();
		}
		else {
			sourceFrame = source->GetFirst(StreamType::Color).get();
			targetFrame = target->GetFirst(StreamType::Color).get();
		}

		calib.Feed(sourceFrame, targetFrame);

		if (calib.ValidPairs() >= minValids) {
			calibrationResult = calib.Calibrate(sourceFrame->GetDescription().intrinsics, targetFrame->GetDescription().intrinsics);
			return calibrationResult;
		}
	}
}

ICPCalibrationResult ICPRefine(RGBDStream::RGBDStreamGroup<RGBDStream::RealsenseStream>& group, glm::mat4 guess) {
	Pointcloud pc0{};
	Pointcloud pc1{};

	auto framesets = group.WaitForSynchronizedFrames();

	Frameset* source = framesets[0].get();
	Frameset* target = framesets[1].get();

	if (source == nullptr || target == nullptr) {
		std::cout << "Could not complete calibration before stream ended.\n";
		return { false };
	}

	pc0.Process(framesets[0].get());
	pc1.Process(framesets[1].get());

	ICPCalibrationResult icpResult = icp(pc0, pc1, GLMToEigen(guess), 0.05, 0.15);
	
	return icpResult;
}

int main() {

	std::string relPath = getPathWindows();

	// Setup Camera

	CameraState cameraState{ {0,0,-3}, {0,1,0}, 90, 0 };

	std::vector<int> inputKeys = { GLFW_KEY_W, GLFW_KEY_A, GLFW_KEY_D, GLFW_KEY_S, GLFW_KEY_UP, GLFW_KEY_LEFT, GLFW_KEY_RIGHT, GLFW_KEY_DOWN };

	// Setup Graphics Pipeline

	GLFWwindow* window = initSimpleResizableViewport(600, 600);

	if (window == nullptr) {
		glfwTerminate();
		return -1;
	}

	IMGUI_CHECKVERSION();
	ImGui::CreateContext();
	ImGui::StyleColorsDark();
	ImGui_ImplGlfw_InitForOpenGL(window, true);
	ImGui_ImplOpenGL3_Init("#version 460");

	Shader pcShader{ (relPath + "\\shaders\\pointcloud.vert").c_str(), (relPath + "\\shaders\\pointcloud.frag").c_str() };
	Shader meshShader{ (relPath + "\\shaders\\mesh.vert").c_str(), (relPath + "\\shaders\\mesh.frag").c_str() };

	glEnable(GL_DEPTH_TEST);

	// Render Loop

	std::unique_ptr<Frameset> fsSource0;
	std::unique_ptr<Frameset> fsSource1;

	double lastTime = glfwGetTime();
	double accumDelta = 0.0;

	AppState appState{};
	std::unique_ptr<RGBDStream::RGBDStreamGroup<RGBDStream::RealsenseStream>> ActivateStreamGroup;

	std::unique_ptr<VoxelState> voxelState;
	std::unique_ptr<PCState> pcState;
	
	while (!glfwWindowShouldClose(window)) {

		if (appState.ReloadRequested) {
			if (appState.UseBag) {
				std::vector<std::string> sources{
					appState.source0BagPath,
					appState.source1BagPath
				};

				RGBDStream::RealsenseStreamConfiguration config{};
				config.UseBag = true;
				ActivateStreamGroup = std::make_unique<RGBDStream::RealsensePlaybackStreamGroup>(sources, std::vector<RGBDStream::RealsenseStreamConfiguration>{ config });

				appState.ReloadRequested = false;
			}
			else {
				std::cout << "Unsupported." << '\n';
				break;
			}
		}

		if (appState.CalibrationRequested) {
			if (appState.UseBag) {
				RGBDStream::RealsenseStreamConfiguration calibConfig{};
				calibConfig.UseBag = true;
				calibConfig.EnableIR = true;

				std::vector<std::string> calibSources{
					appState.source0BagCalibPath,
					appState.source1BagCalibPath
				};
				
				std::unique_ptr<RGBDStream::RGBDStreamGroup<RGBDStream::RealsenseStream>> calibrationCapture = std::make_unique<RGBDStream::RealsensePlaybackStreamGroup>(calibSources, std::vector<RGBDStream::RealsenseStreamConfiguration>{ calibConfig });
			
				StereoCalibrationResult stereoResult = CharucoCalibrate(*calibrationCapture);

				appState.Calibrated = stereoResult.Success;
				appState.LastCalibrationError = stereoResult.Error;
				appState.CalibratedTransform = CVToGLM(stereoResult.R, stereoResult.T);
				appState.CalibrationRequested = false;
			}
			else {
				std::cout << "Unsupported." << '\n';
				break;
			}
		}

		if (appState.ICPCalibrationRequested) {
			if (ActivateStreamGroup == nullptr || !appState.Calibrated) {
				appState.ICPCalibrationRequested = false;
			}
			else {
				ICPCalibrationResult icpResult = ICPRefine(*ActivateStreamGroup, appState.CalibratedTransform);

				appState.ICPRefined = icpResult.Success;
				appState.LastICPCalibrationError = icpResult.Error;
				appState.CalibratedTransform = EigenToGlm(icpResult.Transform.matrix());

				appState.ICPCalibrationRequested = false;
			}
		}

		double currentTime = glfwGetTime();
		double deltaTime = currentTime - lastTime;
		lastTime = currentTime;
		accumDelta += deltaTime;

		for (auto key : inputKeys) {
			if (glfwGetKey(window, key)) {
				cameraState.ProcessInput(key, deltaTime);
			}
		}

		if (ActivateStreamGroup != nullptr && (!appState.PauseMode || (appState.PauseMode && appState.NextRequested))) {
			auto fss = ActivateStreamGroup->WaitForSynchronizedFrames(0);

			if (fss[0] != nullptr || fss[1] != nullptr)
				appState.NextRequested = false;

			if (fss[0] != nullptr) {
				fsSource0 = std::move(fss[0]);
			}
			if (fss[1] != nullptr) {
				fsSource1 = std::move(fss[1]);
			}
		}

		switch (appState.RenderMode) {
		case AppState::RenderMode::PC_BOTH:
		case AppState::RenderMode::PC_1:
		case AppState::RenderMode::PC_0:
			glPointSize(appState.PointSize);
			if (pcState == nullptr) {
				pcState = std::make_unique<PCState>();
			}
			pcState->Update(fsSource0.get(), appState.CalibratedTransform, fsSource1.get(), glm::mat4{1.0f});
			break;
		case AppState::RenderMode::MESH:
			if (voxelState == nullptr || appState.VoxelReloadRequested) {
				voxelState = std::make_unique<VoxelState>();
				voxelState->Grid = std::make_unique<SparseVoxelGrid<8>>(appState.VoxelSize, appState.TSDFTruncation);
				appState.VoxelReloadRequested = false;
			}
			voxelState->Update(fsSource0.get(), appState.CalibratedTransform, fsSource1.get(), glm::mat4{ 1.0f });
			break;
		}

		fsSource0 = nullptr;
		fsSource1 = nullptr;

		glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

		if (appState.WireFrame) {
			glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
		}
		else {
			glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
		}

		switch (appState.RenderMode) {
			case AppState::RenderMode::PC_BOTH:
				if (pcState == nullptr)
					break;
				pcShader.use();
				pcState->Texture0.Bind(0);
				pcShader.setMatrix("mvp", GL_FALSE, glm::value_ptr(cameraState.PerspectiveMatrix() * cameraState.ViewMatrix() * pcState->Source0.Translation));
				pcState->RenderSource0.Draw();
				pcState->Texture1.Bind(0);
				pcShader.setMatrix("mvp", GL_FALSE, glm::value_ptr(cameraState.PerspectiveMatrix()* cameraState.ViewMatrix()* pcState->Source1.Translation));
				pcState->RenderSource1.Draw();
				break;
			case AppState::RenderMode::PC_0:
				if (pcState == nullptr)
					break;
				pcShader.use();
				pcState->Texture0.Bind(0);
				pcShader.setMatrix("mvp", GL_FALSE, glm::value_ptr(cameraState.PerspectiveMatrix() * cameraState.ViewMatrix() * pcState->Source0.Translation));
				pcState->RenderSource0.Draw();
				break;
			case AppState::RenderMode::PC_1:
				if (pcState == nullptr)
					break;
				pcShader.use();
				pcState->Texture1.Bind(0);
				pcShader.setMatrix("mvp", GL_FALSE, glm::value_ptr(cameraState.PerspectiveMatrix() * cameraState.ViewMatrix() * pcState->Source1.Translation));
				pcState->RenderSource1.Draw();
				break;
			case AppState::RenderMode::MESH:
				if (voxelState == nullptr)
					break;
				meshShader.use();
				meshShader.setMatrix("mvp", GL_FALSE, glm::value_ptr(cameraState.PerspectiveMatrix() * cameraState.ViewMatrix()));
				voxelState->RenderSource.Draw();
				break;
		}

		DrawUI(appState, cameraState);

		glfwSwapBuffers(window);
		glfwPollEvents();
	};

	ImGui_ImplOpenGL3_Shutdown();
	ImGui_ImplGlfw_Shutdown();
	ImGui::DestroyContext();

	glfwTerminate();

	return 0;
}