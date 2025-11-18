add_executable(ProcessFootage "processfootage.cpp")
target_include_directories(ProcessFootage PRIVATE)
target_link_libraries(ProcessFootage PRIVATE glm::glm nlohmann_json::nlohmann_json)

