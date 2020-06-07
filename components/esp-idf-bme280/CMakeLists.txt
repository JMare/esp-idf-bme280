cmake_minimum_required(VERSION 3.5)

set(SOURCES bme280_idf.c driver/bme280.c)
idf_component_register(SRCS ${SOURCES}
	INCLUDE_DIRS "." "./driver")

target_compile_options(${COMPONENT_LIB} PRIVATE -Wno-unused-variable -Wno-type-limits -fdiagnostics-color=always)
