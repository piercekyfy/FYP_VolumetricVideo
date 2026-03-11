#ifdef WITH_REALSENSE

#include "RGBDStream/RealsenseStream.hpp"
#include "librealsense2/rs.hpp"

namespace RGBDStream {

	// RS Stream

	struct RealsenseStream::RS_Impl {
		rs2::context Context{};
		rs2::config Config{};
		rs2::pipeline Pipeline{Context};

		Intrinsics ToIntrinsics(const rs2_intrinsics& intrinsics) {
			return {
				intrinsics.width,
				intrinsics.height,
				intrinsics.ppx,
				intrinsics.ppy,
				intrinsics.fx,
				intrinsics.fy,
				{ intrinsics.coeffs[0], intrinsics.coeffs[1], intrinsics.coeffs[2], intrinsics.coeffs[3], intrinsics.coeffs[4] }
			};
		}
	};

	RealsenseStream::RealsenseStream(std::string_view serial, RealsenseStreamConfiguration configuration) : serial(serial), impl(std::make_unique<RS_Impl>()) {
		if (configuration.UseBag) {
			impl->Config.enable_device_from_file(std::string{ serial });
		}
		else {
			impl->Config.enable_device(this->serial);
			impl->Config.enable_stream(RS2_STREAM_COLOR, configuration.Width, configuration.Height, RS2_FORMAT_RGB8, configuration.FPS);
			impl->Config.enable_stream(RS2_STREAM_DEPTH, configuration.Width, configuration.Height, RS2_FORMAT_Z16, configuration.FPS);

			if (configuration.EnableIR)
				impl->Config.enable_stream(RS2_STREAM_INFRARED, 1, configuration.Width, configuration.Height, RS2_FORMAT_Y8, configuration.FPS);
		}
		
		auto profile = impl->Pipeline.start(impl->Config);

		auto colorProfile = profile.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>();
		auto depthProfile = profile.get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>();

		this->description.serial = this->serial;
		this->description.depthScale = profile.get_device().first<rs2::depth_sensor>().get_depth_scale();
		this->description.streams = {
			{StreamType::Color, colorProfile.fps(), 24, impl->ToIntrinsics(colorProfile.get_intrinsics())},
			{StreamType::Depth, depthProfile.fps(), 16, impl->ToIntrinsics(depthProfile.get_intrinsics())}
		};

		auto depthSensor = profile.get_device().first<rs2::depth_sensor>();

		if (configuration.EnableIR) {
			auto irProfile = profile.get_stream(RS2_STREAM_INFRARED).as<rs2::video_stream_profile>();
			this->description.streams.emplace_back(
				StreamType::IR, irProfile.fps(), 8, impl->ToIntrinsics(irProfile.get_intrinsics())
			);
		}

		if (!configuration.UseBag) {
			// Compromises depth quality, but required for IR quality. Recommend to only enable IR when depth isn't used.
			if (configuration.EnableIR) {
				depthSensor.set_option(RS2_OPTION_EMITTER_ENABLED, 0);
			}
			else {
				depthSensor.set_option(RS2_OPTION_EMITTER_ENABLED, 1);
			}
		}
	};

	RealsenseStream::~RealsenseStream() {
		this->Stop();
	}

	void RealsenseStream::Stop() {
		impl->Pipeline.stop();

	}

	std::unique_ptr<Frameset> RealsenseStream::WaitForFrames(int timeout) {
		try {
			rs2::frameset frames = impl->Pipeline.wait_for_frames(timeout);

			auto fs = std::make_unique<Frameset>(this->description);

			rs2::video_frame colorFrame = frames.get_color_frame();
			rs2::depth_frame depthFrame = frames.get_depth_frame();

			try {
				rs2::video_frame irFrame = frames.get_infrared_frame(1);
				cv::Mat irMat(irFrame.get_height(), irFrame.get_width(), CV_8UC1, const_cast<void*>(irFrame.get_data()), static_cast<size_t>(irFrame.get_stride_in_bytes()));
				fs->AddIR(std::make_unique<RGBData>(std::move(irMat)), this->description.GetFirst(StreamType::IR).value(), irFrame.get_timestamp());
			}
			catch (rs2::error) { /* no ir */ }

			int dw = depthFrame.get_width(), dh = depthFrame.get_height();
			auto depthBuf = std::make_unique<uint16_t[]>(dw * dh);
			std::memcpy(depthBuf.get(), depthFrame.get_data(), dw * dh * sizeof(uint16_t));

			cv::Mat colorMat(
				colorFrame.get_height(), colorFrame.get_width(),
				CV_8UC3,
				const_cast<void*>(colorFrame.get_data()),
				static_cast<size_t>(colorFrame.get_stride_in_bytes())
			);
			cv::Mat colorOwned = colorMat.clone();

			fs->AddColor(std::make_unique<RGBData>(std::move(colorOwned)), this->description.GetFirst(StreamType::Color).value(), colorFrame.get_timestamp());
			fs->AddDepth(std::make_unique<DepthData>(std::move(depthBuf), dw, dh, this->description.depthScale), this->description.GetFirst(StreamType::Depth).value(), depthFrame.get_timestamp());

			return fs;
		}
		catch (rs2::error) {
			return nullptr;
		}
		catch (std::exception) {
			throw;
		}
	}

	// RS Stream Group

	std::vector<std::unique_ptr<RealsenseStream>> RealsenseStreamGroup::InitializeStreams(std::vector<std::string> sources, std::vector<RealsenseStreamConfiguration> configurations, bool useBags) {
		if (sources.size() <= 0)
			throw std::invalid_argument("Argument 'bags' size must be >= 1.");

		if (configurations.size() <= 0)
			throw std::invalid_argument("Argument 'configurations' size must be >= 1.");

		std::vector<std::unique_ptr<RealsenseStream>> streams;

		RealsenseStreamConfiguration& configuration = configurations[0];
		for (int i = 0; i < sources.size(); i++) {
			if (configurations.size() > i)
				configuration = configurations[i];

			if (not useBags && configuration.UseBag)
				throw std::invalid_argument("At configuration " + std::to_string(i) + " found Playback (UseBag=true). This is not valid, use RealsensePlaybackStreamGroup instead.");
			else if (useBags && not configuration.UseBag)
				throw std::invalid_argument("At configuration " + std::to_string(i) + " found Live (UseBag=false). This is not valid, use RealsenseStreamGroup instead.");

			auto stream = std::make_unique<RealsenseStream>(sources[i], configuration);
			if (useBags)
				stream->impl->Pipeline.get_active_profile().get_device().as<rs2::playback>().set_real_time(false);
			streams.push_back(std::move(stream));
		}
		return streams;
	}

	RealsenseStreamGroup::RealsenseStreamGroup(std::vector<std::string> serials, std::vector<RealsenseStreamConfiguration> configurations) : streams(std::move(InitializeStreams(serials, configurations, false))) {}

	const FrameDescription& RealsenseStreamGroup::GetDescription(int index) const {
		if (index >= this->streams.size())
			throw std::out_of_range("No stream at index " + std::to_string(index));
		return this->streams[index]->GetDescription();
	}

	const RealsenseStream& RealsenseStreamGroup::GetStream(int index) const {
		if (index >= this->streams.size())
			throw std::out_of_range("No stream at index " + std::to_string(index));
		return *this->streams[index].get();
	}

	std::vector<std::unique_ptr<Frameset>> RealsenseStreamGroup::WaitForFrames(int timeout) {
		std::vector<std::unique_ptr<Frameset>> framesets{};
		framesets.reserve(this->streams.size());

		for (auto& stream : this->streams) {
			framesets.push_back(std::move(stream->WaitForFrames(timeout)));
		}

		return framesets;
	}

	std::vector<std::unique_ptr<Frameset>> RealsenseStreamGroup::WaitForSynchronizedFrames(int timeout, double maxDelta) {
		return this->WaitForFrames(timeout);
	}

	// RS Playback Stream Group

	RealsensePlaybackStreamGroup::RealsensePlaybackStreamGroup(std::vector<std::string> serials, std::vector<RealsenseStreamConfiguration> configurations) : RealsenseStreamGroup(std::move(InitializeStreams(serials, configurations, true))) {}

	std::vector<std::unique_ptr<Frameset>> RealsensePlaybackStreamGroup::WaitForSynchronizedFrames(int timeout, double maxDelta) {
		std::vector<std::unique_ptr<Frameset>> pending(streams.size());

		while (true) {
			for (int i = 0; i < streams.size(); i++) {
				if (!pending[i]) {
					pending[i] = streams[i]->WaitForFrames(timeout);
					std::vector<std::unique_ptr<Frameset>> none;
					none.resize(streams.size());
					if (!pending[i]) return none;
				}
			}

			double maxTs = std::numeric_limits<double>::lowest();
			for (auto& f : pending)
				maxTs = std::max(maxTs, f->GetFirst(StreamType::Depth)->Timestamp);

			bool synced = true;
			for (int i = 0; i < pending.size(); i++) {
				double ts = pending[i]->GetFirst(StreamType::Depth)->Timestamp;
				if (std::abs(ts - maxTs) > maxDelta) {
					synced = false;
					pending[i] = nullptr;
				}
			}

			if (synced)
				return std::move(pending);
		}
	}
}
#endif