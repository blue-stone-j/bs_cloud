# cloud-fit

This is a library to fit geometry shape from 3D point cloud.

Cloud files are stored in folder `assets`. You can get more clouds as example from [here](https://github.com/blue-stone-j/assets).

# function

### 3DLineDetection

This is a dependent and entire package for detecting 3 dimensional line from unorganized point cloud. All details are in this package and they won't be list here.

### cylinder

### line

### plane

Extract plane from cloud and estimate thickness of this plane. Note that the whole input cloud will be use to extract plane.

# how to Use

### cbuild.sh

This shell file is for compilation and installation of code.

1. **multi-thread**: First argument of this shell is num of threads to perform compilation and installation. For example, `make.sh 7` means 7 threads will be used. If no argument is provided, only one thread will be used.
2. **installation directory**: Default installation directory is root directory of this package. You can change `CMAKE_INSTALL_PREFIX` to change the directory.
3. **`BUILD_TEST`**: Default value is `TRUE`, which means executable files for test will be generated. You can change it to `FALSE` if you don't need it. The executable files won't be installed.

### cmake

Compilation configuration is stored in `CMakelists.txt` and cmake file in folder `cmake`.

1. **install_files.cmake**: install header files, generate and install CMakeconfig files.
2. **cloud_fitConfig.cmake.in**: to generate `cloud_fitConfig.cmake`
3. **other cmake file**: compile and install code that fit geometry shape from cloud

### run

1. Run `cbuild.sh` to compile and install.
2. Add runtime path. You can add `export PATH="$PATH":your_path/cmake/cloud_fit`. Replace `your_path` to your installation path, for example `export PATH="$PATH":/home/user/libs/cmake/cloud_fit`
