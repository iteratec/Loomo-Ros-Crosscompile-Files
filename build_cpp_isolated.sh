#!/bin/bash

# Abort script on any failures
set -e

my_loc="$(cd "$(dirname $0)" && pwd)"
source $my_loc/config.sh
source $my_loc/utils.sh

if [ $# == 0 ] || [ $1 == '-h' ] || [ $1 == '--help' ]; then
    echo "Usage: $0 prefix_path"
    echo "  example: $0 /home/user/my_workspace"
    exit 1
fi

cmd_exists catkin_make || die 'catkin_make was not found'
[ "$CMAKE_PREFIX_PATH" = "" ] && die 'could not find target basedir. Have you run build_catkin.sh and sourced setup.bash?'
[ "$RBA_TOOLCHAIN" = "" ] && die 'could not find android.toolchain.cmake, you should set RBA_TOOLCHAIN variable.'

if echo $system | grep _64 >/dev/null; then
    host64='-DANDROID_NDK_HOST_X64=YES'
fi

if [[ $2 == "--debug-symbols" ]] ; then
    CMAKE_BUILD_TYPE=Debug
else
    CMAKE_BUILD_TYPE=Release
fi

prefix=$(cd $1 && pwd)

python=$(which python)
python_lib=/usr/lib/x86_64-linux-gnu/libpython2.7.so
python_inc=/usr/include/python2.7
python2_inc=/usr/include/x86_64-linux-gnu/python2.7

cd $prefix/catkin_ws

echo
echo -e '\e[34mRunning catkin_make.\e[39m'
echo

catkin_make_isolated --quiet --build $prefix/catkin_ws/build --devel $prefix/catkin_ws/devel --install --cmake-args -DCMAKE_TOOLCHAIN_FILE=$RBA_TOOLCHAIN -DANDROID_TOOLCHAIN_NAME=$toolchain -DANDROID_NATIVE_API_LEVEL=$platform $host64 -DPYTHON_EXECUTABLE=$python -DPYTHON_LIBRARY=$python_lib -DPYTHON_INCLUDE_DIR=$python_inc -DPYTHON_INCLUDE_DIR2=$python2_inc -DBUILD_SHARED_LIBS=0 -DCMAKE_INSTALL_PREFIX=$CMAKE_PREFIX_PATH -DBoost_NO_BOOST_CMAKE=ON -DBOOST_ROOT=$CMAKE_PREFIX_PATH -DANDROID=TRUE -DBOOST_INCLUDEDIR=$CMAKE_PREFIX_PATH/include/boost -DBOOST_LIBRARYDIR=$CMAKE_PREFIX_PATH/lib -DROSCONSOLE_BACKEND=print -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}
    #-DBoost_DEBUG=ON \ 
    #-DBoost_NO_BOOST_CMAKE=TRUE -DBoost_NO_SYSTEM_PATHS=TRUE -DBOOST_ROOT:PATHNAME=$CMAKE_PREFIX_PATH/include/boost \
    #-DBOOST_INCLUDEDIR:PATH=$CMAKE_PREFIX_PATH/include/boost -DBOOST_LIBRARYDIR:PATH=$CMAKE_PREFIX_PATH/lib \
    #-DBoost_USE_STATIC_LIBS=ON -DBoost_NO_BOOST_CMAKE=ON

   # -DBOOST_INCLUDEDIR:PATH=$CMAKE_PREFIX_PATH/include -DBOOST_LIBRARYDIR:PATH=$CMAKE_PREFIX_PATH/lib
   # -DBOOST_ROOT:PATHNAME=$CMAKE_PREFIX_PATH/include

echo
echo -e '\e[34mInstalling.\e[39m'
echo
