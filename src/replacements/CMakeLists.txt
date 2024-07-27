if (MINIROS_USE_SYSTEM_PROGRAM_OPTONS)
    find_package(Boost COMPONENTS program_options)

    add_library(impl::program_options ALIAS program_options)
    target_compile_definitions(impl::program_options -DUSE_BOOST_PROGRAM_OPTIONS)
else ()
    add_subdirectory(program_options)
    add_library(impl::program_options ALIAS program_options)
    target_include_directories(program_options INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})
endif()


