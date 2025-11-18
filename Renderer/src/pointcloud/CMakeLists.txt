add_executable(Pointcloud "pointcloud.cpp")
target_include_directories(Pointcloud PRIVATE ${GL_INCLUDES})
target_link_libraries(Pointcloud PRIVATE GL_SHARED glm::glm ${GL_LIBS})

COPY_REL_DIR(Pointcloud shaders/)