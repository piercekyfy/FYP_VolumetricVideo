add_executable(Main "main.cpp")
target_include_directories(Main PRIVATE ${OpenCV_INCLUDE_DIRS} ${GL_INCLUDES})
target_link_libraries(Main PRIVATE glm::glm opencv_core ${GL_LIBS})

copy_opencv_dlls(Main opencv_core)