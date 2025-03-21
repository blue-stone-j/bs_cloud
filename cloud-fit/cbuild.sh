
# colorful output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[0;33m'
BLUE='\033[0;34m'
NC='\033[0m'

# rm -rf build &&
# rm -rf install &&

echo -e "${BLUE}Start working.${NC}"

# set variable
echo -e "${BLUE}Start setting variables.${NC}"
PROJECT_NAME=cloud_fit
PROJECT_NAME_U=$(echo "$PROJECT_NAME" | tr '[:lower:]' '[:upper:]')
PROJECT_NAME_L=$(echo "$PROJECT_NAME" | tr '[:upper:]' '[:lower:]')

# version of library "cloud_fit"
CLOUD_FIT_VERSION_MAJOR=1
CLOUD_FIT_VERSION_MINOR=0
CLOUD_FIT_VERSION_PATCH=0
CLOUD_FIT_VERSION=${CLOUD_FIT_VERSION_MAJOR}.${CLOUD_FIT_VERSION_MINOR}.${CLOUD_FIT_VERSION_PATCH}

BUILD_TEST=TRUE

CMAKE_INSTALL_PREFIX=$(realpath $(dirname "${BASH_SOURCE[0]}"))/install
echo -e "${GREEN}Set variables completed.${NC}"

# build
echo -e "${BLUE}Start building.${NC}"
mkdir -p build &&
cd build &&
cmake -DCMAKE_EXPORT_COMPILE_COMMANDS=ON\
      -DCLOUD_FIT_VERSION=${CLOUD_FIT_VERSION} \
      -DCMAKE_INSTALL_PREFIX=$CMAKE_INSTALL_PREFIX \
      -DBUILD_TEST=$BUILD_TEST \
      ..

if [ ! $? -eq 0 ]; then
  echo -e "${RED}Failed to build.${NC}"
  exit
fi

if [ "$#" -eq 0 ];then
  num_thr=1
else
  num_thr=$1
fi

# make install -j$num_thr
cmake --build . --target install --parallel $num_thr

if [ ! $? -eq 0 ]; then
  echo -e "${RED}Failed to build.${NC}"
  exit
fi

echo -e "${GREEN}Build completed.${NC}"

echo -e "${GREEN}All tasks have been done.${NC}"

