add_library(program_options program_options.cpp
    values.cpp
    options_descriptor.cpp
    command_line.cpp)
target_include_directories(program_options PUBLIC ${MINIROS_INCLUDE_ROOT})
