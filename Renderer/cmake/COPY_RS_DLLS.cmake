function(copy_rs_dlls TARGET_NAME)
	add_custom_command(TARGET ${TARGET_NAME} POST_BUILD
		COMMAND ${CMAKE_COMMAND} -E copy_if_different
			"${realsense2_DIR}/bin/x64/realsense2.dll"
			"$<TARGET_FILE_DIR:${TARGET_NAME}>"
	)
endfunction()