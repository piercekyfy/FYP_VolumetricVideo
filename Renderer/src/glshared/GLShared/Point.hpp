#pragma once

#include "glm/glm.hpp"



#include "Eigen/Core";

#include <RGBDStream/Frameset.hpp>

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
	glm::mat4 Translation{ 1.0f };

	Pointcloud() {}
	const std::vector<Point> Points() const {
		return this->points;
	}
	std::vector<Eigen::Vector4f> EigenPoints() const {
		std::vector<Eigen::Vector4f> eigenPoints;
		eigenPoints.reserve(points.size());

		for (const auto& p : points) {
			eigenPoints.emplace_back(p.position.x, p.position.y, p.position.z, 1.0f);
		}

		return eigenPoints;
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

		auto depth = depthFrame->AsDepth()->GetData();

		for (int v = 0; v < sourceHeight; v++) {
			for (int u = 0; u < sourceWidth; u++) {
				int index = v * sourceWidth + u;
				float z = depth[index] * depthScale;
				if (z <= 0)
					continue;

				glm::vec2 ray = rays[index];

				points.emplace_back(
					glm::vec3{ ray.x * z, -ray.y * z, z }, // image space (y-down) to 3d (y-up)
					glm::vec2{ u / float(sourceWidth), v / float(sourceHeight) }
				);
			}
		}
	}
	
};

#include <small_gicp/ann/kdtree_omp.hpp>
#include <small_gicp/points/point_cloud.hpp>
#include <small_gicp/factors/gicp_factor.hpp>
#include <small_gicp/util/normal_estimation_omp.hpp>
#include <small_gicp/registration/reduction_omp.hpp>
#include <small_gicp/registration/registration.hpp>
#include <small_gicp/util/downsampling_omp.hpp>

struct ICPCalibrationResult {
	bool Success;
	Eigen::Isometry3d Transform;
	double Error;
};

// https://github.com/koide3/small_gicp
ICPCalibrationResult icp(Pointcloud& sourcePC, Pointcloud& targetPC, Eigen::Isometry3d initialGuess, double resolution = 0.05, double maxDistance = 0.1) {
	using namespace small_gicp;

	auto sPoints = sourcePC.EigenPoints();
	auto tPoints = targetPC.EigenPoints();

	int num_threads = 4;
	int num_neighbors = 10;

	auto target = std::make_shared<small_gicp::PointCloud>(tPoints);
	auto source = std::make_shared<small_gicp::PointCloud>(sPoints);

	// Downsampling
	target = voxelgrid_sampling_omp(*target, resolution, num_threads);
	source = voxelgrid_sampling_omp(*source, resolution, num_threads);

	// Create KdTree
	auto target_tree = std::make_shared<KdTree<PointCloud>>(target, KdTreeBuilderOMP(num_threads));
	auto source_tree = std::make_shared<KdTree<PointCloud>>(source, KdTreeBuilderOMP(num_threads));

	// Estimate point covariances
	estimate_covariances_omp(*target, *target_tree, num_neighbors, num_threads);
	estimate_covariances_omp(*source, *source_tree, num_neighbors, num_threads);

	// GICP + OMP-based parallel reduction
	Registration<GICPFactor, ParallelReductionOMP> registration;
	registration.reduction.num_threads = num_threads;
	registration.rejector.max_dist_sq = maxDistance * maxDistance;

	// Align point clouds
	Eigen::Isometry3d init_T_target_source = initialGuess;
	auto result = registration.align(*target, *source, *target_tree, init_T_target_source);

	Eigen::Isometry3d T = result.T_target_source;  // Estimated transformation
	size_t num_inliers = result.num_inliers;       // Number of inlier source points
	Eigen::Matrix<double, 6, 6> H = result.H;      // Final Hessian matrix (6x6)

	return { result.converged, result.T_target_source, result.error };
}