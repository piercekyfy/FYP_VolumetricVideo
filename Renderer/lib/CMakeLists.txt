# OPENGL

find_package(OpenGL REQUIRED)

# GLFW

set(GLFW_DIR ${CMAKE_CURRENT_SOURCE_DIR}/glfw/)
set(GLFW_BUILD_EXAMPLES OFF CACHE INTERNAL "Build the GLFW example programs")
set(GLFW_BUILD_TESTS OFF CACHE INTERNAL "Build the GLFW test programs")
set(GLFW_BUILD_DOCS OFF CACHE INTERNAL "Build the GLFW documentation")
set(GLFW_INSTALL OFF CACHE INTERNAL "Generate installation target")
add_subdirectory(${GLFW_DIR})

# GLAD

set(GLAD_DIR ${CMAKE_CURRENT_SOURCE_DIR}/glad/)
add_library(GLAD ${GLAD_DIR}/src/glad.c)
target_include_directories(GLAD PUBLIC ${GLAD_DIR}/include)

# GL

add_library(GL_ALL INTERFACE)
target_include_directories(GL_ALL INTERFACE ${GLAD_DIR}/include)
target_link_libraries(GL_ALL INTERFACE glfw GLAD OpenGL::GL ${CMAKE_DL_LIBS})

# OPENCV

find_package(OpenCV REQUIRED)

add_library(CV_ALL INTERFACE)
target_include_directories(CV_ALL INTERFACE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(CV_ALL INTERFACE ${OpenCV_LIBS})

# EIGEN

include(FetchContent)
FetchContent_Declare(
  Eigen
  GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
  GIT_TAG master
  GIT_SHALLOW TRUE
  GIT_PROGRESS TRUE)
set(CMAKE_POLICY_DEFAULT_CMP0077 NEW)
set(EIGEN_BUILD_DOC OFF)
set(EIGEN_BUILD_PKGCONFIG OFF)
FetchContent_MakeAvailable(Eigen)

# NLOHMANN/JSON

FetchContent_Declare(json URL https://github.com/nlohmann/json/releases/download/v3.12.0/json.tar.xz)
FetchContent_MakeAvailable(json)

# glm

add_subdirectory(glm/)

# REALSENSE (Optional)

