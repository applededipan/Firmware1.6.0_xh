#px4_add_module(
#	MODULE modules__hybrid
#	MAIN hybrid
#	COMPILE_FLAGS
#	STACK_MAIN 2000
#	STACK_MAX 1000
#	SRCS
#		hybrid.c
#	DEPENDS
#		platforms__common
#		git_ecl
#	)

px4_add_module(
	MODULE modules__hybrid
	MAIN hybrid
	STACK_MAIN 2000
	SRCS
		hybrid_main.cpp
 #	hybrid.c
 

	DEPENDS
		platforms__common
	)
# vim: set noet ft=cmake fenc=utf-8 ff=unix :
