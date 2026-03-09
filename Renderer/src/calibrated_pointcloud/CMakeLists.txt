add_executable(CalibratedPointcloud "calibrated_pointcloud.cpp")
target_include_directories(CalibratedPointcloud PRIVATE ${INCLUDE_DIR})
target_link_libraries(CalibratedPointcloud PRIVATE GL_SHARED GL_ALL CV_ALL RGBD_STREAM glm::glm nlohmann_json::nlohmann_json Eigen3::Eigen)

COPY_RS_DLLS(CalibratedPointcloud)
COPY_OPENCV_DLLS(CalibratedPointcloud ${OpenCV_LIBS})
COPY_REL_DIR(CalibratedPointcloud shaders/)
COPY_REL_DIR(CalibratedPointcloud frames/)