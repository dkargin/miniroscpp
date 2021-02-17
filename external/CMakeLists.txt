
if (NOT MINIROS_USE_SYSTEM_BZIP)
	if (EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/bzip2)
		# Disabling bzip2 application.
		set(ENABLE_LIB_ONLY ON)
		# Dirty hack to make Python3 fail to find. bzip2 will not build tests without Python3.
		# And it will not fail configure tests with ENABLE_LIB_ONLY=ON
		set(Python3_EXECUTABLE "/nowhere/to/found")
		# We will build bzip2 as a static library.
		set(ENABLE_SHARED_LIB OFF)
		set(ENABLE_STATIC_LIB ON)
		add_subdirectory(bzip2)
		set(BZIP2_FOUND TRUE PARENT_SCOPE)
		set(BZIP2_LIBRARIES bz2_static PARENT_SCOPE)
		set(BZIP2_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/bzip2 PARENT_SCOPE)

		set(EXTERNAL_INCLUDE ${EXTERNAL_INCLUDE} ${BZIP2_INCLUDE_DIR} PARENT_SCOPE)
		
		# Adding replacement library for bz2. It does not have any "Interface" headers and we
		# can install it without these headers
		get_directory_property(BZ2_SOURCES DIRECTORY bzip2 DEFINITION BZ2_SOURCES)
		#get_target_property(BZ2_SOURCES bz2_static SOURCES)
		list(TRANSFORM BZ2_SOURCES PREPEND "bzip2/")
		add_library(bz2_import STATIC ${BZ2_SOURCES})
		target_include_directories(bz2_import PRIVATE
			bzip2
			${CMAKE_CURRENT_BINARY_DIR}/bzip2)		
	else()
		message(SEND_ERROR "Missing directory \"${CMAKE_CURRENT_SOURCE_DIR}/bzip2\"")
		message(SEND_ERROR "Please run \"git submodule init --update\" to update submodules")
	endif()
endif()

if (NOT MINIROS_USE_SYSTEM_LZ4)
	if (EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/lz4)
		set (LZ4_BUILD_CLI OFF)
		set (LZ4_BUILD_LEGACY_LZ4C OFF)
		add_subdirectory(lz4/build/cmake)
		set(lz4_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/lz4/lib PARENT_SCOPE)
		set(lz4_LIBRARIES lz4_static PARENT_SCOPE)
		#target_include_directories(lz4_static PRIVATE ${lz4_INCLUDE_DIRS})
		set(EXTERNAL_INCLUDE ${EXTERNAL_INCLUDE} ${lz4_static} PARENT_SCOPE)
		#install(TARGETS lz4_static EXPORT MiniROSTargets DESTINATION lib)
	else()
		message(SEND_ERROR "Missing directory \"${CMAKE_CURRENT_SOURCE_DIR}/lz4\"")
		message(SEND_ERROR "Please run \"git submodule init --update\" to update submodules")
	endif()
endif()
