add_library(RGBD_STREAM STATIC 
	"FileRGBDStream.cpp" "RealsenseRGBDStream.cpp")

target_include_directories(RGBD_STREAM 
	PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

# Note OpenCV dependency. DLLs must be copied using copy_opencv_dlls macro or another method.
target_link_libraries(RGBD_STREAM PRIVATE CV_ALL nlohmann_json::nlohmann_json)

if(TARGET realsense2::realsense2) # Note Realsense dlls must also be copied
	target_link_libraries(RGBD_STREAM PUBLIC realsense2::realsense2)
	target_compile_definitions(RGBD_STREAM PUBLIC WITH_REALSENSE)
endif()