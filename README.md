These scripts will (hopefully) help you build static libraries
for tf2 for android and setup a sample application.

You will need android SDK installed and the 'android' program
location in the $PATH.


## Disclaimer

This project is a prototype developed by iteratec's students and interns.
It is work in progress in many areas and does not reflect the usual code quality standards
applied by iteratec.

## How to

For a detailed how-to please refer to: http://wiki.ros.org/android_ndk

Requirements: Docker

This repository builds the needed .so-Librarys for Loomo-App.
In order to start simply clone the repository and call the script do_docker with the portable flag:

    git clone https://github.com/iteratec/Loomo-Ros-Crosscompile-Files.git
    cd Loomo-Ros-Crosscompile-Files/
    ./do_docker.sh --portable
    
The first exectuion will take a long time (expect 1h+) and takes up to 19GB on the harddrive.
Once it is finshed all needed librarys for the Loomo-App can be found at

    /output/loomo_native_app/libs/armeabi-v7a/
    
These .so-Files now need to be copied to the '/jnilibs/armeabi-v7a/' folder of the Loomo-App.


INSTALL
-------

Source ROS (for python tools):

    source /opt/ros/hydro/setup.bash

The `do_everything.sh` script will call all the other scripts
sequentially, you just have to give it a prefix path:

    ./do_everything.sh /path/to/workspace

YOU WILL PROBABLY HAVE TO RUN THIS MULTIPLE TIMES DUE TO PTHREAD LINKING.

You can also run each script individually, most of them have
a minimalistic help string to give you an idea of their parameters.

When finished, the script will give you a few lines of what it did.
If everything went fine, you will be able to do the following:

    cd /path/to/workspace/sample_app
    ant debug

This will build the app. If you want to install it, run the following:

    ant debug install

This will install the app onto a virtual android device running in the
emulator.

To follow what the app does, you will need to read the log. The sdk has
a tool called `adb` located in `$SDK/platform-tools/`, you can follow the
log by running:

    $SDK/platform-tools/adb logcat

Good luck!
