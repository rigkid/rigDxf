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

# Vendored dxflib (GPL-2.0-or-later) — never formatted, never widened.
set(_dxflib_dir "${CMAKE_CURRENT_LIST_DIR}/third_party/dxflib")
if(NOT TARGET dxflib)
	add_library(dxflib STATIC
		"${_dxflib_dir}/dl_dxf.cpp"
		"${_dxflib_dir}/dl_writer_ascii.cpp"
	)
	target_include_directories(dxflib PUBLIC $<BUILD_INTERFACE:${_dxflib_dir}>)
	set_target_properties(dxflib PROPERTIES POSITION_INDEPENDENT_CODE ON)
endif()

add_library(${PACK_NAME} STATIC
	src/rigDxf.cpp
	src/rigDxf.h
	src/DxfDocument.h
	src/DxfLoader.cpp
	src/DxfLoader.h
)
target_link_libraries(${PACK_NAME} PUBLIC rigkit)
target_link_libraries(${PACK_NAME} PRIVATE dxflib)
if(TARGET rigPlotComponent)
	target_link_libraries(${PACK_NAME} PUBLIC rigPlotComponent)
endif()
# CArc / CEllipse / CSpline / verticesOf — normally reached through rigPlotComponent.
if(TARGET rigComponent)
	target_link_libraries(${PACK_NAME} PUBLIC rigComponent)
endif()
target_include_directories(${PACK_NAME} PUBLIC
	$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/src>
)
target_include_directories(${PACK_NAME} PRIVATE
	${_dxflib_dir}
)

if(EXISTS "${CMAKE_CURRENT_LIST_DIR}/tools/dxf_smoke.cpp")
	add_executable(dxf_smoke "${CMAKE_CURRENT_LIST_DIR}/tools/dxf_smoke.cpp")
	target_link_libraries(dxf_smoke PRIVATE ${PACK_NAME})
endif()

message(STATUS "Configured pack: ${PACK_NAME} (DXF read/write via dxflib)")

