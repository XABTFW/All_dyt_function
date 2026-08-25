px4_add_module(
	MODULE modules__comm_emergency
	MAIN comm_emergency
	SRCS
		comm_emergency.cpp
	DEPENDS
		px4_work_queue
)

px4_add_unit_gtest(SRC CommEmergencyStateMachineTest.cpp)
