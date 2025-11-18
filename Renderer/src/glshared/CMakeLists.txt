add_library(GL_SHARED STATIC "shader.cpp")
target_include_directories(GL_SHARED 
	PUBLIC ${INCLUDE_DIR}/glshared
	PRIVATE ${GL_INCLUDES})
target_link_libraries(GL_SHARED
	PUBLIC glm::glm
	PRIVATE ${GL_LIBS})