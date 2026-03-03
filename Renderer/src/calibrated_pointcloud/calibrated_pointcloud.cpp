#include <iostream>

#include "glm/glm.hpp"
#include "glm/gtc/matrix_transform.hpp"
#include "glm/gtc/type_ptr.hpp"

#include "glcommon.hpp"
#include <RGBDStream/Frameset.hpp>
#include "RGBDStream/FileRGBDStream.hpp"

#include "opencv2/core.hpp"
#include "opencv2/stereo.hpp"
#include "opencv2/calib3d.hpp"

struct Point {
	glm::vec3 position;
	glm::vec2 texcoord;
};

class Pointcloud {
private: 
	int sourceWidth{ -1 };
	int sourceHeight{ -1 };
	std::vector<glm::vec2> rays{};
	std::vector<Point> points{};

	void ComputeRays(Frame* frame) {
		auto desc = frame->GetDescription();

		if (desc.intrinsics.width == sourceWidth && desc.intrinsics.height == sourceHeight)
			return;

		sourceWidth = desc.intrinsics.width;
		sourceHeight = desc.intrinsics.height;

		rays.clear();
		rays.resize(sourceWidth * sourceHeight);

		for (int v = 0; v < sourceHeight; v++) {
			for (int u = 0; u < sourceWidth; u++) {

				rays[v * sourceWidth + u] = glm::vec2{
					float(u - desc.intrinsics.ppx) / desc.intrinsics.fx,
					float(v - desc.intrinsics.ppy) / desc.intrinsics.fy
				};
			}
		}
	}
public:
	Pointcloud() {}
	const std::vector<Point> Points() const {
		return this->points;
	}
	void Process(Frameset* frameset) {
		if (frameset == nullptr || frameset->GetFirst(StreamType::Depth) == nullptr)
			throw std::exception("Invalid frame.");

		auto depthFrame = frameset->GetFirst(StreamType::Depth);
		double depthScale = frameset->GetDescription().depthScale;
		bool hasColor = frameset->Has(StreamType::Color); // TODO: alignment if resolutions don't match

		ComputeRays(depthFrame.get());

		points.clear();

		auto depthIntr = depthFrame->GetDescription().intrinsics;

		auto depth = depthFrame->AsDepth()->data;

		for (int v = 0; v < sourceHeight; v++) {
			for (int u = 0; u < sourceWidth; u++) {
				int index = v * sourceWidth + u;
				float z = depth[index] * depthScale;
				if (z <= 0)
					continue;

				glm::vec2 ray = rays[index];

				points.emplace_back(
					glm::vec3{ ray.x * z, -ray.y * z, z },
					glm::vec2{u / float(sourceWidth), v / float(sourceHeight)}
					);
			}
		}
	}
};

class RenderedPoints {
private:
	GLuint VAO;
	GLuint VBO;
	int size{};
public:
	RenderedPoints() {
		glGenVertexArrays(1, &VAO);
		glGenBuffers(1, &VBO);

		glBindVertexArray(VAO);
		glBindBuffer(GL_ARRAY_BUFFER, VBO);

		glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Point), (void*)0);
		glEnableVertexAttribArray(0);
		glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, sizeof(Point), (void*)offsetof(Point, texcoord));
		glEnableVertexAttribArray(1);

		glBindVertexArray(0);
	}
	~RenderedPoints() {
		if (VBO) glDeleteBuffers(1, &VBO);
		if (VAO) glDeleteVertexArrays(1, &VAO);
	}
	void Update(const std::vector<Point>& buffer) {
		glBindBuffer(GL_ARRAY_BUFFER, VBO);
		glBufferData(GL_ARRAY_BUFFER, buffer.size() * sizeof(Point), buffer.data(), GL_DYNAMIC_DRAW);
		size = buffer.size();
	}
	void Draw() const {
		if (size == 0) return;

		glBindVertexArray(VAO);
		glDrawArrays(GL_POINTS, 0, static_cast<GLsizei>(size));
		glBindVertexArray(0);
	}
};

class RenderTexture {
private:
	GLuint TEX;
	bool allocated = false;
public:
	RenderTexture() {
		glGenTextures(1, &TEX);
		glBindTexture(GL_TEXTURE_2D, TEX);

		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	}
	void Set(const cv::Mat& image) {
		glBindTexture(GL_TEXTURE_2D, TEX);

		if (!allocated) {
			glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, image.cols, image.rows, 0, GL_RGB, GL_UNSIGNED_BYTE, image.data);
			allocated = true;
		}
		else {
			glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, image.cols, image.rows, GL_RGB, GL_UNSIGNED_BYTE, image.data);
		}

		glGenerateMipmap(GL_TEXTURE_2D);
	}
	void Bind(int slot = 0) const {
		glActiveTexture(GL_TEXTURE0 + slot);
		glBindTexture(GL_TEXTURE_2D, TEX);
	}
};

struct StereoPoseResult {
	bool Success;
	cv::Mat R, T, E, F;
	double Error;
	int SuccessfulSamples;
};

class StereoCalibrator {
public:
	StereoCalibrator(int boardWidth, int boardHeight, double cellSize, int imageWidth, int imageHeight) 
		:	boardWidth(boardWidth), boardHeight(boardHeight), cellSize(cellSize), 
			imageWidth(imageWidth), imageHeight(imageHeight) 
	{}

	bool Process(Frame* R, Frame* L) {
		auto rgbR = R->AsColor();
		auto rgbL = L->AsColor();

		if (rgbR == nullptr || rgbL == nullptr)
			throw std::exception("Cannot estimate pose from non-rgb frame.");

		return Feed(rgbR, rgbL);
	}

	StereoPoseResult GetPose(Intrinsics intrR, Intrinsics intrL) {
		return Calibrate(intrR, intrL);
	}

	const int SampleSize() const {
		return this->sampleSize;
	}

private:
	int boardWidth;
	int boardHeight;
	double cellSize;
	int imageWidth{};
	int imageHeight{};
	std::vector<std::vector<cv::Point3f>> objPoints{};
	std::vector<std::vector<cv::Point2f>> imgPointsR, imgPointsL{};
	int sampleSize{ 0 };

	const bool Feed(const RGBData* R, const RGBData* L) {
		cv::Mat imageR{ R->image.clone() };
		cv::Mat imageL{ L->image.clone() };
		cv::cvtColor(imageR, imageR, cv::COLOR_RGB2GRAY);
		cv::cvtColor(imageL, imageL, cv::COLOR_RGB2GRAY);

		std::vector<cv::Point2f> pointsR{};
		std::vector<cv::Point2f> pointsL{};
		if (!FindCheckerboard(imageR, pointsR)) {
			std::cout << "Failed to find source0 checkerboard!";
			return false;
		}
		if (!FindCheckerboard(imageL, pointsL)) {
			std::cout << "Failed to find source1 checkerboard!";
			return false;
		}

		std::vector<cv::Point3f> localObjectPoints{};
		for (size_t i = 0; i < boardHeight; i++) {
			for (size_t j = 0; j < boardWidth; j++) {
				localObjectPoints.emplace_back(j * cellSize, i * cellSize, 0.0f);
			}
		}
		
		objPoints.push_back(localObjectPoints);
		imgPointsR.push_back(pointsR);
		imgPointsL.push_back(pointsL);

		sampleSize++;

		return true;
	}

	const StereoPoseResult Calibrate(Intrinsics intrR, Intrinsics intrL) const {
		if (sampleSize <= 0)
			return { false };

		cv::Mat R, T, E, F;

		double error = cv::stereoCalibrate(
			objPoints,
			imgPointsR,
			imgPointsL,
			GetIntrinsicsMat(intrR),
			GetCoeffsMat(intrR),
			GetIntrinsicsMat(intrL),
			GetCoeffsMat(intrL),
			cv::Size{ imageWidth, imageHeight }, R, T, E, F, cv::CALIB_FIX_INTRINSIC, cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 100, 1e-5));
	
		return { true, R, T, E, F, error, sampleSize }; // Rotation between, translation between, essential matrix, fundament matrix (pixel L -> pixel R)
	}

	const bool FindCheckerboard(cv::Mat image, std::vector<cv::Point2f>& corners) const {
		bool foundCorners = cv::findChessboardCorners(image, cv::Size{ boardWidth, boardHeight }, corners);

		if (!foundCorners) { return false; };

		cv::cornerSubPix(image, corners, cv::Size{ 11, 11 }, cv::Size{ -1, -1 }, cv::TermCriteria{ cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 30, 0.001 });

		return true;
	}

	const cv::Mat GetIntrinsicsMat(Intrinsics intrinsics) const {
		return (cv::Mat_<double>(3, 3) << intrinsics.fx, 0, intrinsics.ppx, 0, intrinsics.fy, intrinsics.ppy, 0, 0, 1);
	}

	const cv::Mat GetCoeffsMat(Intrinsics intrinsics) const {
		return (cv::Mat_<double>(1, 5) << intrinsics.coeffs[0], intrinsics.coeffs[1], intrinsics.coeffs[2], intrinsics.coeffs[3], intrinsics.coeffs[4]);
	}
};

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

	RGBDStream::FileRGBDStream cam0{ relPath + "\\frames\\239622300610" };
	RGBDStream::FileRGBDStream cam1{ relPath + "\\frames\\241122306275" };
	auto rgbDescription = cam0.GetDescription().GetFirst(StreamType::Color).value().get();

	StereoCalibrator calibrator{ 9, 6, 0.0253, rgbDescription.intrinsics.width, rgbDescription.intrinsics.height };

	auto calibrationResult = EstimateStereoPose(cam0, cam1, calibrator);

	cam0.Reset();
	cam1.Reset();

	if (!calibrationResult.Success) {
		std::cerr << "Failed to calibrate any samples. Is there a clearly visible checkerboard pattern in all cameras view?" << '\n';
		return -1;
	}

	std::cout << "Calibrated with " << calibrationResult.SuccessfulSamples << " samples and an error of: " << calibrationResult.Error << " pixels." << '\n';

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

	glm::mat4 alignTransform = CVToGLM(calibrationResult.R, calibrationResult.T);

	while (!glfwWindowShouldClose(window)) {

		double currentTime = glfwGetTime();
		double deltaTime = currentTime - lastTime;
		timer += deltaTime;
		lastTime = currentTime;

		if (fs0 == nullptr || fs1 == nullptr) {
			fs0 = cam0.WaitForFrames();
			fs1 = cam1.WaitForFrames();

			if (fs0 == nullptr || fs1 == nullptr) {
				cam0.Reset();
				cam1.Reset();

				fs0 = cam0.WaitForFrames();
				fs1 = cam1.WaitForFrames();
			}
		}

		if (timer >= 0.3 && fs0 != nullptr && fs1 != nullptr) { // Next Frame

			timer = 0.0;

			tex0.Set(fs0->GetFirst(StreamType::Color)->AsColor()->image);
			tex1.Set(fs1->GetFirst(StreamType::Color)->AsColor()->image);

			pc0.Process(fs0.get());
			pc1.Process(fs1.get());

			r_pc0.Update(pc0.Points());
			r_pc1.Update(pc1.Points());

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

		tex1.Bind(0);

		shader.setMatrix("mvp", GL_FALSE, glm::value_ptr(mvp * glm::inverse(alignTransform)));

		r_pc1.Draw();

		glfwSwapBuffers(window);
		glfwPollEvents();
	};

	glfwTerminate();

	return 0;
}