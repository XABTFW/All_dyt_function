px4_add_module(
	MODULE modules__cooperative_rendezvous
	MAIN cooperative_rendezvous
	SRCS
		cooperative_rendezvous.cpp
	DEPENDS
		px4_work_queue
	)
