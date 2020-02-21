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
