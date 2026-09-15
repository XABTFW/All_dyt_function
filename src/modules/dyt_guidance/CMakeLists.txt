px4_add_module(
	MODULE modules__dyt_guidance
	MAIN dyt_guidance
	SRCS
		dyt_guidance.cpp
	DEPENDS
		px4_work_queue
	)

px4_add_unit_gtest(SRC DytTerminalGuidanceTest.cpp)
