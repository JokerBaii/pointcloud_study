add_library(kdtree STATIC
  kdtree.cpp
  kdtree.hpp
)
add_library(study_PCL::kdtree ALIAS kdtree)

target_include_directories(kdtree
  PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_compile_features(kdtree PUBLIC cxx_std_17)
set_target_properties(kdtree PROPERTIES CXX_EXTENSIONS OFF)
