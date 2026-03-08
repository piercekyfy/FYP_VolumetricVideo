#ifdef WITH_REALSENSE

#include "RGBDStream/RealsenseRGBDStream.hpp"

#include "librealsense2/rs.hpp"

struct RealsenseRGBData : public RGBData {
	rs2::video_frame RSVideoFrame;

	RealsenseRGBData(rs2::video_frame frame) : RSVideoFrame(std::move(frame)) {
		this->image = cv::Mat{
			RSVideoFrame.get_height(),
			RSVideoFrame.get_width(),
			CV_8UC3,
			const_cast<void*>(RSVideoFrame.get_data()),
			static_cast<size_t>(RSVideoFrame.get_stride_in_bytes())
		};
	}
};

struct RealsenseDepthData : public DepthData {
	rs2::depth_frame RSDepthFrame;

	RealsenseDepthData(rs2::depth_frame frame, double depthScale) : RSDepthFrame(std::move(frame)) {
		this->width = RSDepthFrame.get_width();
		this->height = RSDepthFrame.get_height();
		this->scale = depthScale;
	}
	const uint16_t* GetData() const override {
		return reinterpret_cast<const uint16_t*>(RSDepthFrame.get_data());
	}
};

namespace RGBDStream {

	struct RealsenseRGBDStream::RS_Impl {
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

	RealsenseRGBDStream::RealsenseRGBDStream(std::string_view serial, RealsenseStreamConfiguration configuration) : serial(serial), impl(std::make_unique<RS_Impl>()) {
		impl->Config.enable_device(this->serial);
		impl->Config.enable_stream(RS2_STREAM_COLOR, configuration.Width, configuration.Height, RS2_FORMAT_RGB8, configuration.FPS);
		impl->Config.enable_stream(RS2_STREAM_DEPTH, configuration.Width, configuration.Height, RS2_FORMAT_Z16, configuration.FPS);

		auto profile = impl->Pipeline.start(impl->Config);
		auto colorProfile = profile.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>();
		auto depthProfile = profile.get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>();

		this->description.serial = this->serial;
		this->description.depthScale = profile.get_device().first<rs2::depth_sensor>().get_depth_scale();
		this->description.streams = {
			{StreamType::Color, colorProfile.fps(), 24, impl->ToIntrinsics(colorProfile.get_intrinsics())},
			{StreamType::Depth, depthProfile.fps(), 16, impl->ToIntrinsics(depthProfile.get_intrinsics())}
		};
	};

	RealsenseRGBDStream::RealsenseRGBDStream(std::string_view bagPath) : impl(std::make_unique<RS_Impl>()) {
		impl->Config.enable_device_from_file(std::string{ bagPath });
		impl->Config.enable_stream(RS2_STREAM_COLOR, RS2_FORMAT_Z16);
		impl->Config.enable_stream(RS2_STREAM_DEPTH, RS2_FORMAT_RGB8);

		auto profile = impl->Pipeline.start(impl->Config);

		auto colorProfile = profile.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>();
		auto depthProfile = profile.get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>();

		this->description.serial = this->serial;
		this->description.depthScale = profile.get_device().first<rs2::depth_sensor>().get_depth_scale();
		this->description.streams = {
			{StreamType::Color, colorProfile.fps(), 24, impl->ToIntrinsics(colorProfile.get_intrinsics())},
			{StreamType::Depth, depthProfile.fps(), 16, impl->ToIntrinsics(depthProfile.get_intrinsics())}
		};
	}

	void RealsenseRGBDStream::Stop() {
		impl->Pipeline.stop();

	}

	RealsenseRGBDStream::~RealsenseRGBDStream() {
		this->Stop();
	}

	std::unique_ptr<Frameset> RealsenseRGBDStream::WaitForFrames(int timeout) {
		try {
			rs2::frameset frames = impl->Pipeline.wait_for_frames(timeout);

			auto fs = std::make_unique<Frameset>(this->description);

			rs2::video_frame colorFrame = frames.get_color_frame();
			rs2::depth_frame depthFrame = frames.get_depth_frame();

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

			fs->AddColor(std::make_unique<RGBData>(std::move(colorOwned)), this->description.GetFirst(StreamType::Color).value());
			fs->AddDepth(std::make_unique<DepthData>(std::move(depthBuf), dw, dh, this->description.depthScale), this->description.GetFirst(StreamType::Depth).value());

			return fs;
		}
		catch (rs2::error) {
			return nullptr;
		}
		catch (std::exception) {
			throw;
		}
	}
}
#endif