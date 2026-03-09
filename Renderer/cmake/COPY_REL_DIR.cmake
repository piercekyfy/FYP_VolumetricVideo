function(copy_rel_dir TARGET_NAME SOURCE_DIR)
	string(REPLACE "/" "_" DIR_NAME_SAFE ${SOURCE_DIR})
    set(CUSTOM_TARGET_NAME "copy_${TARGET_NAME}_${DIR_NAME_SAFE}")

	add_custom_target(${CUSTOM_TARGET_NAME} ALL
		COMMAND ${CMAKE_COMMAND} -E copy_directory_if_different
			"${CMAKE_CURRENT_SOURCE_DIR}/${SOURCE_DIR}"
			"$<TARGET_FILE_DIR:${TARGET_NAME}>/${SOURCE_DIR}"
		COMMENT "[${TARGET_NAME}] COPY ${SOURCE_DIR}"
	)
endfunction()