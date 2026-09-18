# Catkin extras for find_package(miniros).
#
# Included from the generated minirosConfig.cmake. Downstream packages
# (appro_drivers, miniros_benchmark) use the MINIROS_* names from the
# non-catkin minirosConfig.cmake.in; this file maps catkin's miniros_*
# variables onto that contract.

if(NOT MINIROS_INCLUDE_DIRS)
  set(MINIROS_INCLUDE_DIRS ${miniros_INCLUDE_DIRS})
endif()

# version.h is copied into the devel include tree (see miniros/CMakeLists.txt).
if(miniros_DEVEL_PREFIX)
  list(INSERT MINIROS_INCLUDE_DIRS 0 "${miniros_DEVEL_PREFIX}/include")
endif()

if(NOT MINIROS_GENERATED_INCLUDE_DIRS)
  if(miniros_SOURCE_PREFIX)
    set(MINIROS_GENERATED_INCLUDE_DIRS "${miniros_SOURCE_PREFIX}/include/generated")
  else()
    set(MINIROS_GENERATED_INCLUDE_DIRS "${miniros_PREFIX}/include/generated")
  endif()
endif()

if(NOT MINIROS_LIBRARIES)
  set(MINIROS_LIBRARIES ${miniros_LIBRARIES})
endif()

# Same-workspace catkin consumers link miniros::roscxx. Aliases are created
# while configuring the miniros package; recreate them if find_package() ran
# against an already-built catkin devel/install tree.
if(NOT TARGET miniros::roscxx)
  if(TARGET roscxx)
    add_library(miniros::roscxx ALIAS roscxx)
  elseif(miniros_LIBRARIES)
    add_library(miniros::roscxx INTERFACE IMPORTED)
    set_target_properties(miniros::roscxx PROPERTIES
      INTERFACE_INCLUDE_DIRECTORIES "${MINIROS_INCLUDE_DIRS};${MINIROS_GENERATED_INCLUDE_DIRS}"
      INTERFACE_LINK_LIBRARIES "${miniros_LIBRARIES}")
  endif()
endif()

if(NOT TARGET miniros::bag_storage)
  if(TARGET bag_storage)
    add_library(miniros::bag_storage ALIAS bag_storage)
  endif()
endif()
