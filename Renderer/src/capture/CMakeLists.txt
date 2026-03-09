if(TARGET realsense2::realsense2)
	add_executable(Capture "capture.cpp")
	target_link_libraries(Capture PRIVATE GL_SHARED GL_ALL CV_ALL RGBD_STREAM glm::glm nlohmann_json::nlohmann_json ${OpenCV_LIBS} realsense2::realsense2)

	COPY_RS_DLLS(Capture)
	COPY_OPENCV_DLLS(Capture ${OpenCV_LIBS})
	COPY_REL_DIR(Capture shaders/)
endif()