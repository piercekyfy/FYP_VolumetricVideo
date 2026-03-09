cmake_minimum_required (VERSION 3.8)

# Enable Hot Reload for MSVC compilers if supported.
if (POLICY CMP0141)
  cmake_policy(SET CMP0141 NEW)
  set(CMAKE_MSVC_DEBUG_INFORMATION_FORMAT "$<IF:$<AND:$<C_COMPILER_ID:MSVC>,$<CXX_COMPILER_ID:MSVC>>,$<$<CONFIG:Debug,RelWithDebInfo>:EditAndContinue>,$<$<CONFIG:Debug,RelWithDebInfo>:ProgramDatabase>>")
endif()

if(WIN32)
    set(USER_HOME "$ENV{USERPROFILE}")
else()
    set(USER_HOME "$ENV{HOME}")
endif()

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

include(FetchContent)
include(ExternalProject)



project (Renderer)

set(SRC_DIR ${CMAKE_SOURCE_DIR}/src)
set(INCLUDE_DIR ${CMAKE_SOURCE_DIR}/include)
set(LIB_DIR ${CMAKE_SOURCE_DIR}/lib)

# Had to patch-work opencv and realsense to work

set(OpenCV_DIR ${LIB_DIR}/opencv/)
set(OpenCV_DLLS_DIR ${OpenCV_DIR}/x64/vc18/bin)

set(realsense2_DIR "" CACHE PATH "Optional path to Realsense SDK.")

if(realsense2_DIR)
	
	add_library(realsense2::realsense2 SHARED IMPORTED GLOBAL)
	set_target_properties(realsense2::realsense2 PROPERTIES
		IMPORTED_LOCATION     "${realsense2_DIR}/bin/x64/realsense2.dll"
		IMPORTED_IMPLIB       "${realsense2_DIR}/lib/x64/realsense2.lib"
		INTERFACE_INCLUDE_DIRECTORIES "${realsense2_DIR}/include"
	)
endif()

# deps and scripts

add_subdirectory(cmake/)
add_subdirectory(lib/)

# src

add_subdirectory(src/)