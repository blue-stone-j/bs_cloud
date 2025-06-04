
set(module_name fit)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
set(fit_include ${EIGEN3_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})
set(fit_lib ${PCL_LIBRARIES})

compile_module(${module_name} "${fit_include}" "${fit_lib}")