px4_add_module(
	MODULE drivers__dyt_gimbal
	MAIN dyt_gimbal
	SRCS
		dyt_gimbal.cpp
	DEPENDS
		drivers_rangefinder
	)

px4_add_unit_gtest(SRC DytPixelLosTest.cpp)
