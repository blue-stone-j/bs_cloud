
set(module_name segment)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenMP)
set(segment_include ${EIGEN3_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})
set(segment_lib ${PCL_LIBRARIES})

compile_module(${module_name} "${segment_include}" "${segment_lib}")
