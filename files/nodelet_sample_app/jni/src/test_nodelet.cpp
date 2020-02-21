/**
 * (c) 2016 Ernesto Corbellini, Creativa77 SRL
 * 
 * Demonstrates the use of ROS nodelets in Android.
 * Example that loads the tutorial math nodelet. It listens in the
 * 'in' topic for a value and adds it to an initial value (default zero)
 * and outputs the result in the 'out' topic.
 * 
 **/

#ifndef ROS_MASTER_URI
#error ROS_MASTER_URI MUST be set in files/nodelet_sample_app/jni/Android.mk.in
#endif
#ifndef ROS_ANDROID_IP
#error ROS_ANDROID_IP MUST be set in files/nodelet_sample_app/jni/Android.mk.in
#endif

#include "ros/ros.h"
#include <nodelet/loader.h>
#include <iostream>
#include <string>
#include <arpa/inet.h>
#include <sys/socket.h>
#include "ifaddrs.h"

#include <android_native_app_glue.h>
#include <android/log.h>

#define  LOG_TAG    "NODELET_TEST"

#define  LOGD(...)  __android_log_print(ANDROID_LOG_DEBUG, LOG_TAG, __VA_ARGS__)
#define  LOGE(...)  __android_log_print(ANDROID_LOG_ERROR, LOG_TAG, __VA_ARGS__)

bool getHostIp(const char *interface, char *ipaddr);

int main()
{    
    bool res;
    std::map<std::string, std::string> remappings;
    std::vector<std::string> nodelet_argv;
    char strbuf[128];
    char ipAddr[20];
    char *argv[] = {"cmd", ROS_MASTER_URI, ROS_ANDROID_IP};
    int argc = 3;
    
    // Dynamically obtain the IP of the host we are running in.
    if (!getHostIp("wlan0", ipAddr))
    {
        LOGD("Failed to get IP address for this interface!");
        return 1;
    }
    sprintf(strbuf, "__ip:=%s", ipAddr);
    argv[2] = strbuf;

    LOGD("Starting program...");
    
    ros::init(argc, argv, "simple_nodelet_loader");

    std::string master_uri = ros::master::getURI();

    if (ros::master::check())
    {
        LOGD("ROS master is up at %s", master_uri.c_str());
        LOGD("Local address is %s", ipAddr, ipAddr);
    } else
    {
        LOGD("Failed to find ROS master!");
        return 1;
    }    
    
    ros::NodeHandle nh;

    nodelet::Loader loader(nh);

    //ros::param::set("/test_nodelet/value", 11.4);

    LOGD("Loading nodelet...");        
    res = loader.load("/test_nodelet", "nodelet_tutorial_math/Plus", remappings, nodelet_argv);
    
    if (!res)
    {
        LOGD("Problem loading nodelet!");
        return 1;
    }

    ros::Rate loop_rate(10);
    
    LOGD("Starting ROS main loop...");
    
    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

    LOGD("Program ending...");
    
    return 0;
}

void android_main(struct android_app* state) {
  app_dummy(); // needed so the Android glue does not get stripped off
  main();
}

// Get the current host IP address on the specified interface.
bool getHostIp(const char *interface, char *ipaddr)
{
    struct ifaddrs *ifap, *ifa;
    struct sockaddr_in *sa;
    char *addr = NULL;

    getifaddrs(&ifap);

    for (ifa = ifap; ifa; ifa = ifa->ifa_next)
    {
        if (ifa->ifa_addr && ifa->ifa_addr->sa_family == AF_INET)
        {
            if (strcmp(ifa->ifa_name, interface) == 0)
            {
                sa = (struct sockaddr_in *) ifa->ifa_addr;
                addr = inet_ntoa(sa->sin_addr);
            }
        }
    }
    
    if (!addr) return(false);
    
    strcpy(ipaddr, addr);

    freeifaddrs(ifap);
    return(true);
}
