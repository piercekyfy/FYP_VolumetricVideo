#pragma once

#include "RGBDStream.hpp"
#include "Frameset.hpp"

#include <string_view>
#include <memory>

namespace RGBDStream {

	struct RealsenseStreamConfiguration {
		int Width;
		int Height;
		int FPS;
		bool EnableIR;
		/// <summary>
		/// Use a bag file path in place of connected device serial. Every other configuration but EnableIR will be ignored.
		/// </summary>
		bool UseBag;
	};

	class SyncedRealsenseBags;

	class RealsenseRGBDStream : public RGBDStream {
		friend class SyncedRealsenseBags;
	public:
		RealsenseRGBDStream(std::string_view serial, RealsenseStreamConfiguration configuration);
		RealsenseRGBDStream(std::string_view bagPath);
		~RealsenseRGBDStream();
		std::unique_ptr<Frameset> WaitForFrames(int timeout = 1000) override;
		void Stop();
	protected:
		std::string serial{};
		struct RS_Impl;
		std::unique_ptr<RS_Impl> impl{};
	};

	class SyncedRealsenseBags {
	public:
		SyncedRealsenseBags(
			std::string_view bagA,
			std::string_view bagB,
			RealsenseStreamConfiguration config,
			double maxSyncDeltaMs = 16.0);
		std::pair<std::unique_ptr<Frameset>, std::unique_ptr<Frameset>> WaitForFrames(int timeout = 5000);
	private:
		std::unique_ptr<RealsenseRGBDStream> streamA, streamB;
		std::unique_ptr<Frameset> pendingA, pendingB;
		double maxDeltaMs;
	};
}