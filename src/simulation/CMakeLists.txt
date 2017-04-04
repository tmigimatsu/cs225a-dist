
add_executable(simulator
	${CS225A_COMMON_SOURCE}
	simulation_main.cpp)

target_link_libraries(simulator
	${CS225A_COMMON_LIBRARIES})
