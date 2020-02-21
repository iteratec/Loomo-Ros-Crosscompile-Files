#!/bin/bash

# Abort script on any failures
set -e

my_loc="$(cd "$(dirname $0)" && pwd)"
source $my_loc/config.sh
source $my_loc/utils.sh

if [ $# != 2 ] || [ $1 == '-h' ] || [ $1 == '--help' ]; then
    echo "Usage: $0 project_path portable"
    echo "  example: $0 /home/user/my_workspace/tf2_ndk 0, will use external links"
    echo "  example: $0 /home/user/my_workspace/tf2_ndk 1, will use copy all required files"
    exit 1
fi

[ "$CMAKE_PREFIX_PATH" = "" ] && die 'could not find target basedir. Have you run build_catkin.sh and sourced setup.bash?'

if [ ! -d $1 ]; then
    mkdir -p $1
fi

cd $1

if [[ $2 -eq 0 ]]; then
  ln -fs $CMAKE_PREFIX_PATH/include ./
  ln -fs $CMAKE_PREFIX_PATH/lib ./
  ln -fs $CMAKE_PREFIX_PATH/share ./
else
  cp -r $CMAKE_PREFIX_PATH/include ./
  cp -r $CMAKE_PREFIX_PATH/lib ./
  cp -r $CMAKE_PREFIX_PATH/share ./
fi

cp $my_loc/files/tfa/*.mk ./
