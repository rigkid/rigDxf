file(READ "${CMAKE_CURRENT_LIST_DIR}/pack.json" PACK_JSON)
string(JSON PACK_NAME GET "${PACK_JSON}" name)

if(NOT TARGET rigPlotComponent)
	if(DEFINED RIGKIT_ROOT AND NOT RIGKIT_ROOT STREQUAL "")
		set(_dep "${RIGKIT_ROOT}/packs/rigPlotComponent")
	else()
		set(_dep "${CMAKE_SOURCE_DIR}/packs/rigPlotComponent")
	endif()
	if(EXISTS "${_dep}/CMakeLists.txt")
		add_subdirectory("${_dep}" "${CMAKE_BINARY_DIR}/packs/rigPlotComponent")
	endif()
endif()

add_library(${PACK_NAME} STATIC
	src/rigDxf.cpp
	src/rigDxf.h
	src/DxfLoader.cpp
	src/DxfLoader.h
)
target_link_libraries(${PACK_NAME} PUBLIC rigkit)
if(TARGET rigPlotComponent)
	target_link_libraries(${PACK_NAME} PUBLIC rigPlotComponent)
endif()
target_include_directories(${PACK_NAME} PUBLIC
	$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/src>
)

if(EXISTS "${CMAKE_CURRENT_LIST_DIR}/tools/dxf_smoke.cpp")
	add_executable(dxf_smoke "${CMAKE_CURRENT_LIST_DIR}/tools/dxf_smoke.cpp")
	target_link_libraries(dxf_smoke PRIVATE ${PACK_NAME})
endif()

message(STATUS "Configured pack: ${PACK_NAME} (DXF import)")

