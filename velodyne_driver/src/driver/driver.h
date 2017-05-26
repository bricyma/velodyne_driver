/* -*- mode: C++ -*- */
/*
 *  Copyright (C) 2012 Austin Robot Technology, Jack O'Quin
 * 
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

/** \file
 *
 *  ROS driver interface for the Velodyne 3D LIDARs
 */

#ifndef _VELODYNE_DRIVER_H_
#define _VELODYNE_DRIVER_H_ 1

#include <string>
#include <ros/ros.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>
#include <dynamic_reconfigure/server.h>

#include <velodyne_driver/input.h>
#include <velodyne_driver/VelodyneNodeConfig.h>
#include <velodyne_driver/rawdata.h>
#include <ctime>
#include <std_msgs/Int8.h>


namespace velodyne_driver
{

class VelodyneDriver
{
public:

  VelodyneDriver(ros::NodeHandle node,
                 ros::NodeHandle private_nh);
  ~VelodyneDriver() {}

  bool poll(void);
  int count; //for trigger identidency

private:

  ///Callback for dynamic reconfigure
  void callback(velodyne_driver::VelodyneNodeConfig &config,
              uint32_t level);

  ///Pointer to dynamic reconfigure service srv_
  boost::shared_ptr<dynamic_reconfigure::Server<velodyne_driver::
              VelodyneNodeConfig> > srv_;

  // calculate unixtime from packet         
  void unixTime(velodyne_msgs::VelodynePacket *pkt, int& pkt_year, int& pkt_month, int& pkt_date,
                              int& pkt_hours, int& pkt_minutes, int& pkt_seconds, int& pkt_gps_status);
  // callback for camera sub  
  void trigger_callback(const std_msgs::Int8::ConstPtr &msg);

  // configuration parameters
  struct
  {
    std::string frame_id;            ///< tf frame ID
    std::string model;               ///< device model name
    int    npackets;                 ///< number of packets to collect
    double rpm;                      ///< device rotation rate (RPMs)
    double time_offset;              ///< time in seconds added to each velodyne time stamp
  } config_;

  boost::shared_ptr<Input> input_;
  boost::shared_ptr<my_velodyne_rawdata::RawData> data_;
  ros::Publisher output_;
  ros::Publisher t_output_;  //trigger pub
  ros::Subscriber t_input_;  //trigger sub

  /** diagnostics updater */
  diagnostic_updater::Updater diagnostics_;
  double diag_min_freq_;
  double diag_max_freq_;
  boost::shared_ptr<diagnostic_updater::TopicDiagnostic> diag_topic_;
};

} // namespace velodyne_driver

#endif // _VELODYNE_DRIVER_H_
