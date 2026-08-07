px4_add_module(
	MODULE drivers__sdm50
	MAIN sdm50
	SRCS
		SDM50.cpp
		SDM50.hpp
		sdm50_main.cpp
	DEPENDS
		drivers_rangefinder
)
