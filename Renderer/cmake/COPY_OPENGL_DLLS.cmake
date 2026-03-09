function(copy_opencv_dlls TARGET_NAME)
	string(JOIN ".*|" LIB_SEARCH_STR ${ARGN})
	set(LIB_SEARCH_STR "(${LIB_SEARCH_STR}).*")

	file(GLOB DLLS "${OpenCV_DLLS_DIR}/*.dll")
	foreach(DLL ${DLLS})
		get_filename_component(DLL_NAME "${DLL}" NAME_WE)
		if(DLL_NAME MATCHES ${LIB_SEARCH_STR})
			add_custom_command(TARGET ${TARGET_NAME} POST_BUILD
				COMMAND ${CMAKE_COMMAND} -E copy_if_different
					"${DLL}"
					"$<TARGET_FILE_DIR:${TARGET_NAME}>"
			)
		endif()
	endforeach()
endfunction()