add_executable(FootagePointcloud "footage_pointcloud.cpp")
target_include_directories(FootagePointcloud PRIVATE ${OpenCV_INCLUDE_DIRS} ${GL_INCLUDES})
target_link_libraries(FootagePointcloud PRIVATE GL_SHARED glm::glm nlohmann_json::nlohmann_json ${OpenCV_LIBS} ${GL_LIBS})

COPY_OPENCV_DLLS(FootagePointcloud ${OpenCV_LIBS})
COPY_REL_DIR(FootagePointcloud shaders/)
COPY_REL_DIR(FootagePointcloud frames/)