add_library(RGBD_STREAM STATIC 
	"FileRGBDStream.cpp" "RealsenseRGBDStream.cpp")

target_include_directories(RGBD_STREAM 
	PUBLIC ${INCLUDE_DIR} ${OpenCV_INCLUDE_DIRS})

# Note OpenCV dependency. DLLs must be copied using copy_opencv_dlls macro or another method.
target_link_libraries(RGBD_STREAM PUBLIC ${OpenCV_Libs})
target_link_libraries(RGBD_STREAM PRIVATE nlohmann_json::nlohmann_json)

if(TARGET realsense2::realsense2) # Note Realsense dlls must also be copied
	target_link_libraries(RGBD_STREAM PUBLIC realsense2::realsense2)
	target_compile_definitions(RGBD_STREAM PUBLIC WITH_REALSENSE)
endif()