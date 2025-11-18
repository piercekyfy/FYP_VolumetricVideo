cmake_minimum_required (VERSION 3.8)

# Enable Hot Reload for MSVC compilers if supported.
if (POLICY CMP0141)
  cmake_policy(SET CMP0141 NEW)
  set(CMAKE_MSVC_DEBUG_INFORMATION_FORMAT "$<IF:$<AND:$<C_COMPILER_ID:MSVC>,$<CXX_COMPILER_ID:MSVC>>,$<$<CONFIG:Debug,RelWithDebInfo>:EditAndContinue>,$<$<CONFIG:Debug,RelWithDebInfo>:ProgramDatabase>>")
endif()

include(FetchContent)

project (Renderer)

set(SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src)
set(INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include)
set(LIB_DIR ${CMAKE_CURRENT_SOURCE_DIR}/lib)
set(SOURCES
	${SRC_DIR}/main.cpp
)

# Macros

function(copy_rel_dir TARGET_NAME SOURCE_DIR)
	add_custom_command(TARGET ${TARGET_NAME} POST_BUILD
		COMMAND ${CMAKE_COMMAND} -E copy_directory
			"${CMAKE_CURRENT_SOURCE_DIR}/${SOURCE_DIR}"
			"$<TARGET_FILE_DIR:${TARGET_NAME}>/${SOURCE_DIR}"
	)
endfunction()

# OpenGL
find_package(OpenGL REQUIRED)
include_directories(${OPENGL_INCLUDE_DIRS})

# GLFW
set(GLFW_DIR ${LIB_DIR}/glfw)
set(GLFW_BUILD_EXAMPLES OFF CACHE INTERNAL "Build the GLFW example programs")
set(GLFW_BUILD_TESTS OFF CACHE INTERNAL "Build the GLFW test programs")
set(GLFW_BUILD_DOCS OFF CACHE INTERNAL "Build the GLFW documentation")
set(GLFW_INSTALL OFF CACHE INTERNAL "Generate installation target")
add_subdirectory(${GLFW_DIR})

set(GLAD_DIR ${LIB_DIR}/glad)
add_library(GLAD ${GLAD_DIR}/src/glad.c)
target_include_directories(GLAD PRIVATE ${GLAD_DIR}/include)

# opencv

set(OpenCV_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../opencv)
set(OpenCV_DLLS_DIR ${OpenCV_DIR}/x64/vc17/bin)

find_package(OpenCV REQUIRED)

function(copy_opencv_dlls TARGET_NAME)
	string(JOIN ".*|" LIB_SEARCH_STR ${ARGN})
	set(LIB_SEARCH_STR "(${LIB_SEARCH_STR}).*")

	file(GLOB DLLS "${OpenCV_DLLS_DIR}/*.dll")
	foreach(DLL ${DLLS})
		get_filename_component(DLL_NAME "${DLL}" NAME_WE)
		message(STATUS "${DLL_NAME}")
		if(DLL_NAME MATCHES ${LIB_SEARCH_STR})
			add_custom_command(TARGET ${TARGET_NAME} POST_BUILD
				COMMAND ${CMAKE_COMMAND} -E copy_if_different
					"${DLL}"
					"$<TARGET_FILE_DIR:${TARGET_NAME}>"
			)
		endif()
	endforeach()
endfunction()

# glm

add_subdirectory(${LIB_DIR}/glm)

# Define common OpenGL dependencies

SET(GL_INCLUDES
	${GLAD_DIR}/include
	${OPENGL_INCLUDE_DIRS}
)

SET(GL_LIBS
	glfw
	GLAD 
	${OPENGL_LIBRARIES}
	${CMAKE_DL_LIBS}
)

# Sources

add_subdirectory(${SRC_DIR})

#if (CMAKE_VERSION VERSION_GREATER 3.12)
#  set_property(TARGET LearnOpenGL PROPERTY CXX_STANDARD 20)
#endif()
