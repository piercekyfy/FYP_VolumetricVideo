add_executable(Calibrate "calibrate.cpp")
target_link_libraries(Calibrate PRIVATE GL_SHARED GL_ALL CV_ALL RGBD_STREAM glm::glm nlohmann_json::nlohmann_json)

COPY_OPENCV_DLLS(Calibrate ${OpenCV_LIBS})
COPY_REL_DIR(Calibrate frames/)