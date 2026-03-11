add_library(GL_SHARED STATIC "shader.cpp")
target_include_directories(GL_SHARED 
	PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(GL_SHARED
	PUBLIC glm::glm GL_ALL SMALL_GICP)