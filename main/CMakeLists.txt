cmake_minimum_required(VERSION 3.5)

file(GLOB SOURCES *.c)
idf_component_register(SRCS ${SOURCES}
	INCLUDE_DIRS "."
  REQUIRES esp-idf-bme280)

target_compile_options(${COMPONENT_LIB} PRIVATE -fdiagnostics-color=always)
