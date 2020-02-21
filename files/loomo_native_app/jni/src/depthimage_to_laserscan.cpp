/*
 * Copyright (c) 2012, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/* 
 * Author: Chad Rockey
 */

#include <depthimage_to_laserscan.h>
#include <android/log.h>
#include <jni.h>

using std::string;
using namespace  depthimage_to_laserscan;

inline string stdStringFromjString(JNIEnv *env, jstring java_string) {
    const char *tmp = env->GetStringUTFChars(java_string, NULL);
    string out(tmp);
    env->ReleaseStringUTFChars(java_string, tmp);
    return out;
}

void log(const char *msg, ...) {
    va_list args;
    va_start(args, msg);
    __android_log_vprint(ANDROID_LOG_INFO, "DepthImageToLaserScanROS.cpp", msg, args);
    va_end(args);
}

// DepthimageToLaserScanROS

DepthImageToLaserScanROS::DepthImageToLaserScanROS(ros::NodeHandle& n, ros::NodeHandle& pnh):pnh_(pnh), it_(n), srv_(pnh) {
  boost::mutex::scoped_lock lock(connect_mutex_);
  
  log("Starting Reconfigure");
  // Dynamic Reconfigure
  dynamic_reconfigure::Server<depthimage_to_laserscan::DepthConfig>::CallbackType f;
  f = boost::bind(&DepthImageToLaserScanROS::reconfigureCb, this, _1, _2);
  srv_.setCallback(f);
  
  log("Preparing lazy subscription");
  // Lazy subscription to depth image topic
  pub_ = n.advertise<sensor_msgs::LaserScan>("scan", 10, boost::bind(&DepthImageToLaserScanROS::connectCb, this, _1), boost::bind(&DepthImageToLaserScanROS::disconnectCb, this, _1));
  log("Exiting Ctor");
}

DepthImageToLaserScanROS::~DepthImageToLaserScanROS(){
  sub_.shutdown();
}

void DepthImageToLaserScanROS::depthCb(const sensor_msgs::ImageConstPtr& depth_msg,
        const sensor_msgs::CameraInfoConstPtr& info_msg){
  try
  {
    sensor_msgs::LaserScanPtr scan_msg = dtl_.convert_msg(depth_msg, info_msg);
    pub_.publish(scan_msg);
  }
  catch (std::runtime_error& e)
  {
    log("Failed depth conversion");
    ROS_ERROR_THROTTLE(1.0, "Could not convert depth image to laserscan: %s", e.what());
  }
}

void DepthImageToLaserScanROS::connectCb(const ros::SingleSubscriberPublisher& pub) {
  boost::mutex::scoped_lock lock(connect_mutex_);
  if (!sub_ && pub_.getNumSubscribers() > 0) {
    ROS_DEBUG("Connecting to depth topic.");
    log("Connecting to depth topic.");
    image_transport::TransportHints hints("raw", ros::TransportHints(), pnh_);
    log("Created image_transport hints.");
    sub_ = it_.subscribeCamera("image", 10, &DepthImageToLaserScanROS::depthCb, this, hints);
    log("Subscribed to Camera images");
  }
}

void DepthImageToLaserScanROS::disconnectCb(const ros::SingleSubscriberPublisher& pub) {
  boost::mutex::scoped_lock lock(connect_mutex_);
  if (pub_.getNumSubscribers() == 0) {
    ROS_DEBUG("Unsubscribing from depth topic.");
    log("Unsubscribing from depth topic.");
    sub_.shutdown();
  }
}

void DepthImageToLaserScanROS::reconfigureCb(depthimage_to_laserscan::DepthConfig& config, uint32_t level){
    dtl_.set_scan_time(config.scan_time);
    dtl_.set_range_limits(config.range_min, config.range_max);
    dtl_.set_scan_height(config.scan_height);
    dtl_.set_output_frame(config.output_frame_id);
    dtl_.set_filter(config.filter);
    dtl_.set_kernel_size(config.kernel_size);
    dtl_.set_kernel_type(config.kernel_type);
    dtl_.set_image_offset(config.image_offset);
}

// DepthimageToLaserScanROS

// DepthimageToLaserScan

#include <cv_bridge/cv_bridge.h>

using namespace depthimage_to_laserscan;
  
//DepthImageToLaserScan::DepthImageToLaserScan(){}

DepthImageToLaserScan::~DepthImageToLaserScan(){
}

double DepthImageToLaserScan::magnitude_of_ray(const cv::Point3d& ray) const{
  return sqrt(pow(ray.x, 2.0) + pow(ray.y, 2.0) + pow(ray.z, 2.0));
}

double DepthImageToLaserScan::angle_between_rays(const cv::Point3d& ray1, const cv::Point3d& ray2) const{
  double dot_product = ray1.x*ray2.x + ray1.y*ray2.y + ray1.z*ray2.z;
  double magnitude1 = magnitude_of_ray(ray1);
  double magnitude2 = magnitude_of_ray(ray2);;
  return acos(dot_product / (magnitude1 * magnitude2));
}

bool DepthImageToLaserScan::use_point(const float new_value, const float old_value, const float range_min, const float range_max) const{  
  // Check for NaNs and Infs, a real number within our limits is more desirable than these.
  bool new_finite = std::isfinite(new_value);
  bool old_finite = std::isfinite(old_value);
  
  // Infs are preferable over NaNs (more information)
  if(!new_finite && !old_finite){ // Both are not NaN or Inf.
    if(!isnan(new_value)){ // new is not NaN, so use it's +-Inf value.
      return true;
    }
    return false; // Do not replace old_value
  }
  
  // If not in range, don't bother
  bool range_check = range_min <= new_value && new_value <= range_max;
  if(!range_check){
    return false;
  }
  
  if(!old_finite){ // New value is in range and finite, use it.
    return true;
  }
  
  // Finally, if they are both numerical and new_value is closer than old_value, use new_value.
  bool shorter_check = new_value < old_value;
  return shorter_check;
}

const float DepthImageToLaserScan::depth_to_point(const float new_value, const float old_value, const float range_min, const float range_max) const{  
  // Check for NaNs and Infs, a real number within our limits is more desirable than these.
  bool new_finite = std::isfinite(new_value);
  bool old_finite = std::isfinite(old_value);
  
  // Infs are preferable over NaNs (more information)
  if(!new_finite && !old_finite){ // Both are not NaN or Inf.
    if(!isnan(new_value)){ // new is not NaN, so use it's +-Inf value.
      return new_value;
    }
    else return std::numeric_limits<float>::min(); // Replace NaN for laser_filter
  }
  
  // If not in range, don't bother
  bool range_check = range_min <= new_value && new_value <= range_max;
  if(!range_check){
    return std::numeric_limits<float>::min();
  }
  
  if(!old_finite){ // New value is in range and finite, use it.
    return new_value;
  }
  
  // Finally, if they are both numerical and new_value is closer than old_value, use new_value.
  bool shorter_check = new_value < old_value;
  const float value = shorter_check ? new_value : old_value; 
  return value;
}

sensor_msgs::LaserScanPtr DepthImageToLaserScan::convert_msg(const sensor_msgs::ImageConstPtr& depth_msg,
        const sensor_msgs::CameraInfoConstPtr& info_msg){
  // Set camera model
  cam_model_.fromCameraInfo(info_msg);
  
  // Calculate angle_min and angle_max by measuring angles between the left ray, right ray, and optical center ray
  cv::Point2d raw_pixel_left(0, cam_model_.cy());
  cv::Point2d rect_pixel_left = cam_model_.rectifyPoint(raw_pixel_left);
  cv::Point3d left_ray = cam_model_.projectPixelTo3dRay(rect_pixel_left);
  
  cv::Point2d raw_pixel_right(depth_msg->width-1, cam_model_.cy());
  cv::Point2d rect_pixel_right = cam_model_.rectifyPoint(raw_pixel_right);
  cv::Point3d right_ray = cam_model_.projectPixelTo3dRay(rect_pixel_right);
  
  cv::Point2d raw_pixel_center(cam_model_.cx(), cam_model_.cy());
  cv::Point2d rect_pixel_center = cam_model_.rectifyPoint(raw_pixel_center);
  cv::Point3d center_ray = cam_model_.projectPixelTo3dRay(rect_pixel_center);
  
  double angle_max = angle_between_rays(left_ray, center_ray);
  double angle_min = -angle_between_rays(center_ray, right_ray); // Negative because the laserscan message expects an opposite rotation of that from the depth image
  
  // Fill in laserscan message
  sensor_msgs::LaserScanPtr scan_msg(new sensor_msgs::LaserScan());
  scan_msg->header = depth_msg->header;
  if(output_frame_id_.length() > 0){
    scan_msg->header.frame_id = output_frame_id_;
  }
  scan_msg->angle_min = angle_min;
  scan_msg->angle_max = angle_max;
  scan_msg->angle_increment = (scan_msg->angle_max - scan_msg->angle_min) / (depth_msg->width - 1);
  scan_msg->time_increment = 0.0;
  scan_msg->scan_time = scan_time_;
  scan_msg->range_min = range_min_;
  scan_msg->range_max = range_max_;
  
  // Check scan_height vs image_height
  if(scan_height_/2 > cam_model_.cy() || scan_height_/2 > depth_msg->height - cam_model_.cy()){
    std::stringstream ss;
    ss << "scan_height ( " << scan_height_ << " pixels) is too large for the image height.";
    throw std::runtime_error(ss.str());
  }

  // Calculate and fill the ranges
  uint32_t ranges_size = depth_msg->width;
  scan_msg->ranges.assign(ranges_size, std::numeric_limits<float>::quiet_NaN());

  if (filter_)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try 
    { 
      cv_ptr = cv_bridge::toCvCopy( depth_msg);
    }
    catch (cv_bridge::Exception& e)
    {
      std::cout << "Failed to copy depth_msg into Mat:" << e.what() << '\n';
    }

    // apply_filter(cv_ptr , filter_);
    cv::Mat src, dst;
    src = cv_ptr->image;
    if(!filter_==0){
            dst = src.clone();
             
            switch(filter_) 
            {

              case 1: // homogeneous blur
                for ( int i = 1; i < kernel_size_; i = i + 2 )
                { cv::blur( src, dst, cv::Size( i, i ), cv::Point(-1,-1));}
                break;
              case 2:
                /* median filter */
                for ( int i = 1; i < kernel_size_; i = i + 2 )
                { cv::medianBlur ( src, dst, i );}
                break;
              case 3:
              /* gaussian blur*/
                for ( int i = 1; i < kernel_size_; i = i + 2 )
                { cv::GaussianBlur( src, dst, cv::Size( i, i ), 0, 0 ); }
              break;
            }

            cv_ptr->image = dst;
    }else{
            cv_ptr->image = src;
    }
    // apply_filter
    
    sensor_msgs::ImagePtr filtered_depth_msg = cv_ptr->toImageMsg();

    if (filtered_depth_msg->encoding == sensor_msgs::image_encodings::TYPE_16UC1)
    {
      convert<uint16_t>(filtered_depth_msg, cam_model_, scan_msg, scan_height_, image_offset_);
    }
    else if (filtered_depth_msg->encoding == sensor_msgs::image_encodings::TYPE_32FC1)
    {
      convert<float>(filtered_depth_msg, cam_model_, scan_msg, scan_height_, image_offset_);
    }
    else
    {
      std::stringstream ss;
      ss << "Depth image has unsupported encoding: " << filtered_depth_msg->encoding;
      throw std::runtime_error(ss.str());
    }
  }
  else
  {
    if (depth_msg->encoding == sensor_msgs::image_encodings::TYPE_16UC1)
    {
      convert<uint16_t>(depth_msg, cam_model_, scan_msg, scan_height_, image_offset_);
    }
    else if (depth_msg->encoding == sensor_msgs::image_encodings::TYPE_32FC1)
    {
      convert<float>(depth_msg, cam_model_, scan_msg, scan_height_, image_offset_);
    }
    else
    {
      std::stringstream ss;
      ss << "Depth image has unsupported encoding: " << depth_msg->encoding;
      throw std::runtime_error(ss.str());
    }

  }
  
  return scan_msg;
}
/*
void apply_filter(const cv_bridge::CvImagePtr & cv_ptr, const int & filter)
{
  cv::Mat src, dst;
  src = cv_ptr->image;
  dst = src.clone();
  switch(filter) 
  {
    case 1: // homogeneous blur
      for ( int i = 1; i < kernel_size_; i = i + 2 )
      { cv::blur( src, dst, cv::Size( i, i ), cv::Point(-1,-1));}
      break;
    case 2: // median filter 
      for ( int i = 1; i < kernel_size_; i = i + 2 )
      { cv::medianBlur ( src, dst, i );}
      break;
    case 3: // gaussian blur 
      for ( int i = 1; i < kernel_size_; i = i + 2 )
      { cv::GaussianBlur( src, dst, cv::Size( i, i ), 0, 0 ); }
    break;
  }
  cv_ptr->image = dst;
}
*/

/*
void homogeneous_blur()

void median_filter()

void gaussian_blur()
*/

void DepthImageToLaserScan::set_scan_time(const float scan_time){
  scan_time_ = scan_time;
}

void DepthImageToLaserScan::set_range_limits(const float range_min, const float range_max){
  range_min_ = range_min;
  range_max_ = range_max;
}

void DepthImageToLaserScan::set_scan_height(const int scan_height){
  scan_height_ = scan_height;
}

void DepthImageToLaserScan::set_output_frame(const std::string output_frame_id){
  output_frame_id_ = output_frame_id;
}

void DepthImageToLaserScan::set_filter(const int filter){
  filter_ = filter;
}
void DepthImageToLaserScan::set_kernel_size(const int kernel_size){
  kernel_size_ = kernel_size;
}

void DepthImageToLaserScan::set_kernel_type(const int kernel_type){
  kernel_type_ = kernel_type;
}
void DepthImageToLaserScan::set_image_offset(const int image_offset){
  image_offset_ = image_offset;
}

// DepthimageToLaserScan

extern "C"
JNIEXPORT jint JNICALL
Java_de_iteratec_loomo_ros_nodes_DepthimageToLaserScanNode_execute(
    JNIEnv *env,
    jobject /* this */, jstring rosMasterUri, jstring rosHostname, jstring rosNodeName,
    jobjectArray remappingArguments) {
    string master("__master:=" + stdStringFromjString(env, rosMasterUri));
    string hostname("__ip:=" + stdStringFromjString(env, rosHostname));
    string node_name(stdStringFromjString(env, rosNodeName));
    jsize len = env->GetArrayLength(remappingArguments);
    int argc = 0;
    const int static_params = 4;
    std::string ni = "move_base_jni";
    char **argv = new char *[static_params + len];
    argv[argc++] = const_cast<char *>(ni.c_str());
    argv[argc++] = const_cast<char *>(master.c_str());
    argv[argc++] = const_cast<char *>(hostname.c_str());
    char **refs = new char *[len];
    for (int i = 0; i < len; i++) {
        refs[i] = (char *) env->GetStringUTFChars(
                      (jstring) env->GetObjectArrayElement(remappingArguments, i), NULL);
        argv[argc] = refs[i];
        argc++;
    }
    ros::init(argc, &argv[0], node_name.c_str());
    for (int i = 0; i < len; i++) {
        env->ReleaseStringUTFChars((jstring) env->GetObjectArrayElement(remappingArguments, i),
                                   refs[i]);
    }
    delete refs;
    delete argv;
    ros::NodeHandle n;
    ros::NodeHandle pnh("~");
    log("execute of dtl called");
    DepthImageToLaserScanROS dtl(n, pnh);
    log("MUCH SUCCESS");

    ros::spin();
    return 0;
}

extern "C"
JNIEXPORT jint JNICALL
Java_de_iteratec_loomo_ros_nodes_DepthimageToLaserScanNode_shutdown
(JNIEnv *, jobject) {
    log("Shutting down native node.");
    ros::shutdown();

    return 0;
}
