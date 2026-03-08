add_executable(CalibratedPointcloud "calibrated_pointcloud.cpp")
target_include_directories(CalibratedPointcloud PRIVATE ${OpenCV_INCLUDE_DIRS} ${GL_INCLUDES})
target_link_libraries(CalibratedPointcloud PRIVATE GL_SHARED RGBD_STREAM glm::glm nlohmann_json::nlohmann_json ${OpenCV_LIBS} ${GL_LIBS})

COPY_RS_DLLS(CalibratedPointcloud)
COPY_OPENCV_DLLS(CalibratedPointcloud ${OpenCV_LIBS})
COPY_REL_DIR(CalibratedPointcloud shaders/)
COPY_REL_DIR(CalibratedPointcloud frames/)