/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
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
*   * Neither the name of the Willow Garage nor the names of its
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
*********************************************************************/

//! \author Vijay Pradeep

#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/message_filter.h"
#include "sensor_msgs/msg/point_cloud.hpp"
#include "message_filters/subscriber.h"

#include <deque>

// Service
#include "laser_assembler/srv/assemble_scans.hpp"

#include "math.h"

namespace laser_assembler
{

/**
 * \brief Maintains a history of point clouds and generates an aggregate point cloud upon request
 * \todo Clean up the doxygen part of this header
 *
 * @section parameters ROS Parameters
 *
 * Reads the following parameters from the parameter server
 *  - \b "~tf_cache_time_secs" (double) - The cache time (in seconds) to holds past transforms
 *  - \b "~tf_tolerance_secs (double) - The time (in seconds) to wait after the transform for scan_in is available.
 *  - \b "~max_scans" (unsigned int) - The number of scans to store in the assembler's history, until they're thrown away
 *  - \b "~fixed_frame" (string) - The frame to which received data should immeadiately be transformed to
 *  - \b "~downsampling_factor" (int) - Specifies how often to sample from a scan. 1 preserves all the data. 3 keeps only 1/3 of the points.
 *
 *  @section services ROS Service Calls
 *  - \b "~build_cloud" (AssembleScans.srv) - Accumulates scans between begin time and
 *              end time and returns the aggregated data as a point cloud
 */
template<class T>
class BaseAssemblerSrv
{
public:
  BaseAssemblerSrv(std::shared_ptr<rclcpp::Node> nh) ;
  ~BaseAssemblerSrv() ;

  /**
   * \brief Tells the assembler to start listening to input data
   * It is possible that a derived class needs to initialize and configure
   * various things before actually beginning to process scans. Calling start
   * create subcribes to the input stream, thus allowing the scanCallback and
   * ConvertToCloud to be called. Start should be called only once.
   */
  void start() ;


  /** \brief Returns the number of points in the current scan
   * \param scan The scan for for which we want to know the number of points
   * \return the number of points in scan
   */
  virtual unsigned int GetPointsInScan(const T& scan) = 0 ;

  /** \brief Converts the current scan into a cloud in the specified fixed frame
   *
   * Note: Once implemented, ConvertToCloud should NOT catch TF exceptions. These exceptions are caught by
   * BaseAssemblerSrv, and will be counted for diagnostic information
   * \param fixed_frame_id The name of the frame in which we want cloud_out to be in
   * \param scan_in The scan that we want to convert
   * \param cloud_out The result of transforming scan_in into a cloud in frame fixed_frame_id
   */
  virtual void ConvertToCloud(const std::string& fixed_frame_id, const T& scan_in, sensor_msgs::msg::PointCloud& cloud_out) = 0 ;

protected:
  tf2_ros::Buffer* tf2_buffer_;
  tf2_ros::TransformListener* tf2_ ;

  // ros::NodeHandle private_ns_;
  std::shared_ptr<rclcpp::Node> n_;

private:
  // ROS Input/Ouptut Handling
  rclcpp::Service<laser_assembler::srv::AssembleScans>::SharedPtr cloud_srv_server_;
  message_filters::Subscriber<T> scan_sub_;
  tf2_ros::MessageFilter<T>* tf2_filter_;
  message_filters::Connection tf2_filter_connection_;

  //! \brief Callback function for every time we receive a new scan
  //void scansCallback(const tf2_ros::MessageNotifier<T>::MessagePtr& scan_ptr, const T& testA)
  void scansCallback(const std::shared_ptr<const T>& scan_ptr) ;

  //! \brief Service Callback function called whenever we need to build a cloud
  bool buildCloud(laser_assembler::srv::AssembleScans::Request& req, laser_assembler::srv::AssembleScans::Response& resp) ;


  //! \briefsrv:: Stores history of scans
  std::deque<sensor_msgs::msg::PointCloud> scan_hist_ ;
  std::mutex scan_hist_mutex_ ;

  //! \brief The number points currently in the scan history
  unsigned int total_pts_ ;

  //! \brief The max number of scans to store in the scan history
  unsigned int max_scans_ ;

  //! \brief The frame to transform data into upon receipt
  std::string fixed_frame_ ;

  //! \brief How long we should wait before processing the input data. Very useful for laser scans.
  double tf_tolerance_secs_ ;

  //! \brief Specify how much to downsample the data. A value of 1 preserves all the data. 3 would keep 1/3 of the data.
  unsigned int downsample_factor_ ;

} ;

template <class T>
BaseAssemblerSrv<T>::BaseAssemblerSrv(std::shared_ptr<rclcpp::Node> nh) : n_(nh)
{
  // **** Initialize TransformListener ****
  double tf_cache_time_secs ;
  // private_ns_.param("tf_cache_time_secs", tf_cache_time_secs, 10.0) ;
  n_->declare_parameter("tf_cache_time_secs", 10.0);
  tf_cache_time_secs = n_->get_parameter("tf_cache_time_secs").as_double();
  if (tf_cache_time_secs < 0)
    RCLCPP_ERROR(n_->get_logger(), "Parameter tf_cache_time_secs<0 (%f)", tf_cache_time_secs) ;

  tf2_buffer_ = new tf2_ros::Buffer(n_->get_clock(), tf2::durationFromSec(tf_cache_time_secs));
  tf2_ = new tf2_ros::TransformListener(*tf2_buffer_, n_);
  RCLCPP_INFO(n_->get_logger(), "TF Cache Time: %f Seconds", tf_cache_time_secs) ;

  // ***** Set max_scans *****
  const int default_max_scans = 400 ;
  int tmp_max_scans ;
  // private_ns_.param("max_scans", tmp_max_scans, default_max_scans);
  n_->declare_parameter("max_scans", default_max_scans);
  tmp_max_scans = n_->get_parameter("max_scans").as_int();
  if (tmp_max_scans < 0)
  {
    RCLCPP_ERROR(n_->get_logger(), "Parameter max_scans<0 (%i)", tmp_max_scans) ;
    tmp_max_scans = default_max_scans ;
  }
  max_scans_ = tmp_max_scans ;
  RCLCPP_INFO(n_->get_logger(), "Max Scans in History: %u", max_scans_) ;
  total_pts_ = 0 ;    // We're always going to start with no points in our history

  // ***** Set fixed_frame *****
  // private_ns_.param("fixed_frame", fixed_frame_, std::string("ERROR_NO_NAME"));
  n_->declare_parameter("fixed_frame", std::string("ERROR_NO_NAME"));
  fixed_frame_ = n_->get_parameter("fixed_frame").as_string();
  RCLCPP_INFO(n_->get_logger(), "Fixed Frame: %s", fixed_frame_.c_str()) ;
  if (fixed_frame_ == "ERROR_NO_NAME")
    RCLCPP_ERROR(n_->get_logger(), "Need to set parameter fixed_frame") ;

  // ***** Set downsample_factor *****
  int tmp_downsample_factor ;
  // private_ns_.param("downsample_factor", tmp_downsample_factor, 1);
  n_->declare_parameter("downsample_factor", 1);
  tmp_downsample_factor = n_->get_parameter("downsample_factor").as_int();
  if (tmp_downsample_factor < 1)
  {
    RCLCPP_ERROR(n_->get_logger(), "Parameter downsample_factor<1: %i", tmp_downsample_factor) ;
    tmp_downsample_factor = 1 ;
  }
  downsample_factor_ = tmp_downsample_factor ;
  RCLCPP_INFO(n_->get_logger(), "Downsample Factor: %u", downsample_factor_) ;

  // ***** Start Services *****
  // cloud_srv_server_ = private_ns_.advertiseService("build_cloud", &BaseAssemblerSrv<T>::buildCloud, this);
  cloud_srv_server_ = n_->create_service<laser_assembler::srv::AssembleScans>("~/build_cloud", 
        std::bind(&BaseAssemblerSrv<T>::buildCloud, this, std::placeholders::_1, std::placeholders::_2));

  // **** Get the TF Notifier Tolerance ****
  // private_ns_.param("tf_tolerance_secs", tf_tolerance_secs_, 0.0);
  n_->declare_parameter("tf_tolerance_secs", 0.0);
  tf_tolerance_secs_ = n_->get_parameter("tf_tolerance_secs").as_double();
  if (tf_tolerance_secs_ < 0)
    RCLCPP_ERROR(n_->get_logger(), "Parameter tf_tolerance_secs<0 (%f)", tf_tolerance_secs_) ;
  RCLCPP_INFO(n_->get_logger(), "tf Tolerance: %f seconds", tf_tolerance_secs_) ;

  // ***** Start Listening to Data *****
  // (Well, don't start listening just yet. Keep this as null until we actually start listening, when start() is called)
  tf2_filter_ = NULL;

}

template <class T>
void BaseAssemblerSrv<T>::start()
{
  RCLCPP_INFO(n_->get_logger(), "Starting to listen on the input stream") ;
  if (tf2_filter_)
    RCLCPP_ERROR(n_->get_logger(), "assembler::start() was called twice!. This is bad, and could leak memory") ;
  else
  {
    scan_sub_.subscribe(*n_, "scan_in", 10);
    tf2_filter_ = new tf2_ros::MessageFilter<T>(scan_sub_, *tf2_, fixed_frame_, 10);
    tf2_filter_->setTolerance(tf2::durationFromSec(tf_tolerance_secs_));
    tf2_filter_->registerCallback( std::bind(&BaseAssemblerSrv<T>::scansCallback, this, std::placeholders::_1) );
  }
}

template <class T>
BaseAssemblerSrv<T>::~BaseAssemblerSrv()
{
  if (tf2_filter_)
    delete tf2_filter_;

  delete tf2_ ;
  delete tf2_buffer_;
}

template <class T>
void BaseAssemblerSrv<T>::scansCallback(const std::shared_ptr<const T>& scan_ptr)
{
  const T scan = *scan_ptr ;

  sensor_msgs::msg::PointCloud cur_cloud ;

  // Convert the scan data into a universally known datatype: PointCloud
  try
  {
    ConvertToCloud(fixed_frame_, scan, cur_cloud) ;              // Convert scan into a point cloud
  }
  catch(tf2::TransformException& ex)
  {
    RCLCPP_WARN(n_->get_logger(), "Transform Exception %s", ex.what()) ;
    return ;
  }

  // Add the current scan (now of type PointCloud) into our history of scans
  scan_hist_mutex_.lock() ;
  if (scan_hist_.size() == max_scans_)                           // Is our deque full?
  {
    total_pts_ -= scan_hist_.front().points.size() ;            // We're removing an elem, so this reduces our total point count
    scan_hist_.pop_front() ;                                     // The front of the deque has the oldest elem, so we can get rid of it
  }
  scan_hist_.push_back(cur_cloud) ;                              // Add the newest scan to the back of the deque
  total_pts_ += cur_cloud.points.size() ;                       // Add the new scan to the running total of points

  //printf("Scans: %4u  Points: %10u\n", scan_hist_.size(), total_pts_) ;

  scan_hist_mutex_.unlock() ;
}

template <class T>
bool BaseAssemblerSrv<T>::buildCloud(laser_assembler::srv::AssembleScans::Request& req, laser_assembler::srv::AssembleScans::Response& resp)
{
  //printf("Starting Service Request\n") ;

  scan_hist_mutex_.lock() ;
  // Determine where in our history we actually are
  unsigned int i = 0 ;

  // Find the beginning of the request. Probably should be a search
  while ( i < scan_hist_.size() &&                                                    // Don't go past end of deque
          rclcpp::Time(scan_hist_[i].header.stamp) < rclcpp::Time(req.begin) )                                    // Keep stepping until we've exceeded the start time
  {
    i++ ;
  }
  unsigned int start_index = i ;

  unsigned int req_pts = 0 ;                                                          // Keep a total of the points in the current request
  // Find the end of the request
  while ( i < scan_hist_.size() &&                                                    // Don't go past end of deque
          rclcpp::Time(scan_hist_[i].header.stamp) < rclcpp::Time(req.end) )                                      // Don't go past the end-time of the request
  {
    req_pts += (scan_hist_[i].points.size()+downsample_factor_-1)/downsample_factor_ ;
    i += downsample_factor_ ;
  }
  unsigned int past_end_index = i ;

  if (start_index == past_end_index)
  {
    resp.cloud.header.frame_id = fixed_frame_ ;
    resp.cloud.header.stamp = req.end ;
    resp.cloud.points.resize(0) ;
    resp.cloud.channels.resize(0) ;
  }
  else
  {
    // Note: We are assuming that channel information is consistent across multiple scans. If not, then bad things (segfaulting) will happen
    // Allocate space for the cloud
    resp.cloud.points.resize( req_pts ) ;
    const unsigned int num_channels = scan_hist_[start_index].channels.size() ;
    resp.cloud.channels.resize(num_channels) ;
    for (i = 0; i<num_channels; i++)
    {
      resp.cloud.channels[i].name = scan_hist_[start_index].channels[i].name ;
      resp.cloud.channels[i].values.resize(req_pts) ;
    }
    //resp.cloud.header.stamp = req.end ;
    resp.cloud.header.frame_id = fixed_frame_ ;
    unsigned int cloud_count = 0 ;
    for (i=start_index; i<past_end_index; i+=downsample_factor_)
    {
      for(unsigned int j=0; j<scan_hist_[i].points.size(); j+=downsample_factor_)
      {
        resp.cloud.points[cloud_count].x = scan_hist_[i].points[j].x ;
        resp.cloud.points[cloud_count].y = scan_hist_[i].points[j].y ;
        resp.cloud.points[cloud_count].z = scan_hist_[i].points[j].z ;
        for (unsigned int k=0; k<num_channels; k++)
          resp.cloud.channels[k].values[cloud_count] = scan_hist_[i].channels[k].values[j] ;

        cloud_count++ ;
      }
      resp.cloud.header.stamp = scan_hist_[i].header.stamp;
    }
  }
  scan_hist_mutex_.unlock() ;

  RCLCPP_DEBUG(n_->get_logger(), "Point Cloud Results: Aggregated from index %u->%u. BufferSize: %lu. Points in cloud: %lu", start_index, past_end_index, scan_hist_.size(), resp.cloud.points.size()) ;
  return true ;
}

}
