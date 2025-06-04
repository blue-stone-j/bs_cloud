
set(module_name sample)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
set(sample_include ${EIGEN3_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})
set(sample_lib ${PCL_LIBRARIES})

compile_module(${module_name} "${sample_include}" "${sample_lib}")