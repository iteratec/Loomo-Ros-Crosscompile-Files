/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2013, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of Willow Garage, Inc. nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
* Author: Perrine Aguiar
*********************************************************************/

#include <boost/shared_ptr.hpp>

#include <image_transport/image_transport.h>
#include <image_transport/publisher_plugin.h>
#include <pluginlib/class_loader.h>

#include <android_native_app_glue.h>
#include <android/log.h>

#define  LOG_TAG    "PLUGIN_TEST"

#define  LOGD(...)  __android_log_print(ANDROID_LOG_DEBUG, LOG_TAG, __VA_ARGS__)
#define  LOGI(...)  __android_log_print(ANDROID_LOG_INFO, LOG_TAG, __VA_ARGS__)
#define  LOGE(...)  __android_log_print(ANDROID_LOG_ERROR, LOG_TAG, __VA_ARGS__)

int main();

void android_main(struct android_app* state) {
  app_dummy(); // needed so the Android glue does not get stripped off
  main();
}

int main()
{
  LOGD("Starting app.");
  
  pluginlib::ClassLoader<image_transport::PublisherPlugin> pub_loader("image_transport", "image_transport::PublisherPlugin");

  try
  {
    boost::shared_ptr<image_transport::PublisherPlugin> pub_raw = pub_loader.createInstance("image_transport/raw_pub");
    LOGI("Succesfully created raw image transport publisher");
  }
  catch(pluginlib::LibraryLoadException& ex)
  {
    LOGD("The raw image transport publisher plugin failed to load for some reason. Error: %s", ex.what());
  }
  catch(pluginlib::CreateClassException& ex)
  {
    LOGD("Failed to create raw image transport publisher. Error: %s", ex.what());
  }

  try
  {
    boost::shared_ptr<image_transport::PublisherPlugin> pub_compressed = pub_loader.createInstance("image_transport/compressed_pub");
    LOGI("Succesfully created compressed image transport publisher");
  }
  catch(pluginlib::LibraryLoadException& ex)
  {
    LOGD("The compressed image transport plugin failed to load for some reason. Error: %s", ex.what());
  }
  catch(pluginlib::CreateClassException& ex)
  {
    LOGD("Failed to create compressed image transport publisher. Error: %s", ex.what());
  }

  return 0;
}
