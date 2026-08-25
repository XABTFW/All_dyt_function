px4_add_module(
	MODULE modules__gnss_emergency
	MAIN gnss_emergency
	SRCS
		gnss_emergency.cpp
	DEPENDS
		px4_work_queue
)

px4_add_unit_gtest(SRC GnssEmergencyStateMachineTest.cpp)
