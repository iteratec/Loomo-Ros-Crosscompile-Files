## Loomo-Ros-Crosscompile-Files

In order to not rely on a seperate computer besides Loomo, we need to crosscompile certain ROS Packages for them to work with java.
To achieve this we use the [ROS Android NDK](http://wiki.ros.org/android_ndk). 
For each ROS-Node there is a .cpp file in [loomo_native_app](https://github.com/iteratec/Loomo-Ros-Crosscompile-Files/tree/master/files/loomo_native_app/jni/src). They map the methods from the java-nodes of the Loomo-App to their C++ equivalent. Each node needs to have their own Library.

### How to

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

### Disclaimer

This project is a prototype developed by iteratec's students and interns.
It is work in progress in many areas and does not reflect the usual code quality standards
applied by iteratec.

### License

This project is licensed under the Apache 2.0 License - see the [LICENSE.md](LICENSE.md) file for details
