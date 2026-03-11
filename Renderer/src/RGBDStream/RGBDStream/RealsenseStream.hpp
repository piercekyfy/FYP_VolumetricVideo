#pragma once

#include "RGBDStream.hpp"
#include "RGBDStreamGroup.hpp"
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

	class RealsenseStreamGroup;

	class RealsenseStream : public RGBDStream {
		friend class RealsenseStreamGroup;
		friend class RealsensePlaybackStreamGroup;
	public:
		RealsenseStream(std::string_view serial, RealsenseStreamConfiguration configuration);
		~RealsenseStream();
		std::unique_ptr<Frameset> WaitForFrames(int timeout = 1000) override;
		void Stop();
	protected:
		std::string serial{};
		struct RS_Impl;
		std::unique_ptr<RS_Impl> impl{};
	};

	class RealsenseStreamGroup : public RGBDStreamGroup<RealsenseStream> {
	public:
		RealsenseStreamGroup(std::vector<std::string> serials, std::vector<RealsenseStreamConfiguration> configurations);
		const FrameDescription& GetDescription(int index) const override;
		const RealsenseStream& GetStream(int index) const override;
		std::vector<std::unique_ptr<Frameset>> WaitForFrames(int timeout = 1000) override;
		virtual std::vector<std::unique_ptr<Frameset>> WaitForSynchronizedFrames(int timeout = 1000, double maxDelta = 50) override;
	protected:
		RealsenseStreamGroup(std::vector<std::unique_ptr<RealsenseStream>> streams) : streams(std::move(streams)) {}
		std::vector<std::unique_ptr<RealsenseStream>> streams;
		static std::vector<std::unique_ptr<RealsenseStream>> InitializeStreams(std::vector<std::string> sources, std::vector<RealsenseStreamConfiguration> configurations, bool useBags);
	};

	class RealsensePlaybackStreamGroup : public RealsenseStreamGroup {
	public:
		RealsensePlaybackStreamGroup(std::vector<std::string> bagPaths, std::vector<RealsenseStreamConfiguration> configurations);
		std::vector<std::unique_ptr<Frameset>> WaitForSynchronizedFrames(int timeout = 1000, double maxDelta = 50) override;
	};
}