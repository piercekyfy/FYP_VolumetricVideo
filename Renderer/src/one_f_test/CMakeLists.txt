add_executable(OneFTesting "one_f_test.cpp")
target_include_directories(OneFTesting PRIVATE ${INCLUDE_DIR})
target_link_libraries(OneFTesting PRIVATE GL_SHARED GL_ALL CV_ALL RGBD_STREAM glm::glm nlohmann_json::nlohmann_json imgui Eigen3::Eigen)

COPY_RS_DLLS(OneFTesting)
COPY_OPENCV_DLLS(OneFTesting ${OpenCV_LIBS})
COPY_REL_DIR(OneFTesting shaders/)