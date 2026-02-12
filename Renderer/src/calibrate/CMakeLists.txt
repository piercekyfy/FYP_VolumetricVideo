add_executable(Calibrate "calibrate.cpp")
target_include_directories(Calibrate PRIVATE ${OpenCV_INCLUDE_DIRS} ${GL_INCLUDES})
target_link_libraries(Calibrate PRIVATE GL_SHARED RGBD_STREAM glm::glm nlohmann_json::nlohmann_json ${OpenCV_LIBS} ${GL_LIBS})

COPY_OPENCV_DLLS(Calibrate ${OpenCV_LIBS})
COPY_REL_DIR(Calibrate frames/)