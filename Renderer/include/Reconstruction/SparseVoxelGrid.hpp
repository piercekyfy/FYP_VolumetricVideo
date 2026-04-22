#pragma once


// https://www.open3d.org/docs/latest/tutorial/t_reconstruction_system/voxel_block_grid.html
// First, divide 3D spare into sprase block grids. Then, each block grid is further divided into dense voxels.
// W. Dong, Y.Lao, M.Kaess and V.Koltun: ASH: A Modern Framework for Parallel Spatial Hashing in 3D Perception, arXiv, 2021.
// This groups adjacent voxels together, reducing the number of more expensive hash look-ups involved in finding individual voxels

#include "RGBDStream/Frameset.hpp"

#include "glm/glm.hpp"

#include <unordered_map>
#include <vector>
#include <cmath>
#include <algorithm>

struct Voxel {
	float signedDistance = 0.0f; 
	float weight = 0.0f;
	glm::vec3 color = glm::vec3(0.0f);
};

struct VoxelBlockCoordinate {
	int x, y, z;

	bool operator==(const VoxelBlockCoordinate& o) const
	{
		return x == o.x && y == o.y && z == o.z;
	}
};

struct VoxelBlockCoordinateHash {
	size_t operator()(const VoxelBlockCoordinate& c) const
	{
		// https://www.boost.org/doc/libs/1_35_0/doc/html/boost/hash_combine_id241013.html
		// This is boost's hash algorithm. The 0x9e3779b9 magic number attempts to space values more to avoid collisions and bucketing.
		size_t seed = 0;

		size_t hx = std::hash<int>{}(c.x);
		seed ^= hx + 0x9e3779b9 + (seed << 6) + (seed >> 2);
		size_t hy = std::hash<int>{}(c.y);
		seed ^= hy + 0x9e3779b9 + (seed << 6) + (seed >> 2);
		size_t hz = std::hash<int>{}(c.z);
		seed ^= hz + 0x9e3779b9 + (seed << 6) + (seed >> 2);

		return seed;
	}
};

template<int VoxelBlockSize>
class SparseVoxelGrid {
public:	
	struct VoxelBlock {
		Voxel voxels[VoxelBlockSize][VoxelBlockSize][VoxelBlockSize];
	};
	std::unordered_map<VoxelBlockCoordinate, VoxelBlock, VoxelBlockCoordinateHash> blocks;
private:
	float voxelSize;
	float truncationDistance; // The truncation band is how far from the surface the SDF algorithm stops writing true SDF values.

	int FloorDiv(int a, int b) const // c++ integer division will truncate towards 0 e.g. -3 / 2 == -1, when floor(-3 / 2) == -2.
	{
		int q = a / b;
		int r = a % b;

		if (r != 0 && ((a < 0) != (b < 0)))
			--q;

		return q;
	}

	int FloorMod(int a, int b) const
	{
		int r = a % b;
		if (r < 0)
			r += b;
		return r;
	}
public:
	SparseVoxelGrid(float voxelSize, float truncationDistance) : voxelSize(voxelSize), truncationDistance(truncationDistance) {}

	glm::ivec3 GetVoxelPosition(const glm::vec3& worldPosition) const {
		return glm::ivec3(
			static_cast<int>(std::floor(worldPosition.x / voxelSize)),
			static_cast<int>(std::floor(worldPosition.y / voxelSize)),
			static_cast<int>(std::floor(worldPosition.z / voxelSize))
		);
	}

	VoxelBlockCoordinate GetVoxelBlock(const glm::ivec3& voxelPosition, glm::ivec3& localOut) const {
		auto floorDiv = [](int a, int b) -> int { // weird division voodoo. C++ will drop fractions when dividing with integers
			return a / b - (a % b != 0 && (a ^ b) < 0);
			};

		VoxelBlockCoordinate coordinate{ // the entire block's coordinate
			FloorDiv(voxelPosition.x, VoxelBlockSize),
			FloorDiv(voxelPosition.y, VoxelBlockSize),
			FloorDiv(voxelPosition.z, VoxelBlockSize)
		};

		localOut = glm::ivec3( // the specific local coordinate of voxelPosition
			FloorMod(voxelPosition.x, VoxelBlockSize),
			FloorMod(voxelPosition.y, VoxelBlockSize),
			FloorMod(voxelPosition.z, VoxelBlockSize)
		);

		return coordinate;
	}

	Voxel& GetVoxel(const glm::vec3& worldPosition) {
		glm::ivec3 position = GetVoxelPosition(worldPosition);
		glm::ivec3 localPosition;
		VoxelBlockCoordinate blockCoordinate = GetVoxelBlock(position, localPosition);
		VoxelBlock& block = blocks[blockCoordinate];
		return block.voxels[localPosition.x][localPosition.y][localPosition.z];
	}

	glm::vec3 GetCentre(const VoxelBlockCoordinate& coordinate, const glm::ivec3& localPosition) {
		return glm::vec3(
			(coordinate.x * VoxelBlockSize + localPosition.x + 0.5f) * voxelSize,
			(coordinate.y * VoxelBlockSize + localPosition.y + 0.5f) * voxelSize,
			(coordinate.z * VoxelBlockSize + localPosition.z + 0.5f) * voxelSize
		);
	}

	const Voxel* TryGetVoxel(const glm::ivec3& voxelPosition) const
	{
		glm::ivec3 localPosition;
		VoxelBlockCoordinate blockCoordinate = GetVoxelBlock(voxelPosition, localPosition);

		auto it = blocks.find(blockCoordinate);
		if (it == blocks.end()) return nullptr;

		const Voxel& voxel = it->second.voxels[localPosition.x][localPosition.y][localPosition.z];
		if (voxel.weight <= 0.0f) return nullptr;

		return &voxel;
	}

	float TruncationDistance() const {
		return this->truncationDistance;
	}

	float VoxelSize() const {
		return this->voxelSize;
	}

	void Clear() {
		for (auto& [coord, block] : blocks) {
			for (int lz = 0; lz < VoxelBlockSize; ++lz)
				for (auto& x : block.voxels)
					for (auto& y : x)
						for (auto& v : y)
							v = Voxel{};
		}
	}
};

template <int VoxelBlockSize>
void Integrate(SparseVoxelGrid<VoxelBlockSize>& grid, Frameset* frameset, const glm::mat4& transform) {
	double depthScale = frameset->GetDescription().depthScale;
	std::shared_ptr<Frame> colorFrame = frameset->GetFirst(StreamType::Color);
	std::shared_ptr<Frame> depthFrame = frameset->GetFirst(StreamType::Depth);
	cv::Mat colorImage = colorFrame->AsColor()->image;
	const DepthData* depthData = depthFrame->AsDepth();
	Intrinsics depthIntr = depthFrame->GetDescription().intrinsics;

	for (int v = 0; v < depthData->height; v++) {
		for (int u = 0; u < depthData->width; u++) {
			int index = v * depthData->width + u;
			float depth = static_cast<float>(depthData->GetData()[index] * depthScale);
			if (depth <= 0)
				continue;

			float weight = 1.0f;

			glm::vec3 rayDirection = glm::normalize(glm::vec3{
				(u - depthIntr.ppx) / depthIntr.fx,
				-(v - depthIntr.ppy) / depthIntr.fy,
				1.0f
				});

			float zStart = std::max(depth - grid.TruncationDistance(), grid.VoxelSize()); // Only march in the truncation band.
			float zEnd = depth + grid.TruncationDistance();

			int colorU = static_cast<int>((float)u / depthIntr.width * colorImage.cols);
			int colorV = static_cast<int>((float)v / depthIntr.height * colorImage.rows);
			cv::Vec3b rgb = colorImage.at<cv::Vec3b>(colorV, colorU);
			glm::vec3 color{ rgb[0] / 255.0f, rgb[1] / 255.0f, rgb[2] / 255.0f };//rgb[2] / 255.0f, rgb[1] / 255.0f, rgb[0] / 255.0f };

			for (float z = zStart; z <= zEnd; z += grid.VoxelSize()) { // update voxels as we pass the ray
				glm::vec3 cameraPosition = rayDirection * (z / rayDirection.z);
				glm::vec3 worldPosition = glm::vec3(transform * glm::vec4(cameraPosition, 1.0f));

				float sdf = std::clamp(depth - z, -grid.TruncationDistance(), grid.TruncationDistance());

				Voxel& voxel = grid.GetVoxel(worldPosition);

				// Blend new voxel values:
				voxel.signedDistance = (voxel.weight * voxel.signedDistance + weight * sdf) / (voxel.weight + weight);
				voxel.color = (voxel.weight * voxel.color + weight * color) / (voxel.weight + weight);
				voxel.weight = voxel.weight + weight;
			}
		}
	}
}