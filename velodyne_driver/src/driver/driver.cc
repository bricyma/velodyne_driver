/*
 *  Copyright (C) 2007 Austin Robot Technology, Patrick Beeson
 *  Copyright (C) 2009-2012 Austin Robot Technology, Jack O'Quin
 * 
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

/** \file
 *
 *  ROS driver implementation for the Velodyne 3D LIDARs
 */

#include <string>
#include <cmath>

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <velodyne_msgs/VelodyneScan.h>
#include "driver.h"

namespace velodyne_driver
{

VelodyneDriver::VelodyneDriver(ros::NodeHandle node,
                               ros::NodeHandle private_nh)
{
  count = 0;

  // data_ = new my_velodyne_rawdata::RawData();
  // int b = data_->setup(private_nh);
  // std::cout<<"bb*********"<<b<<std::endl;
  // use private node handle to get parameters
  private_nh.param("frame_id", config_.frame_id, std::string("velodyne"));
  std::string tf_prefix = tf::getPrefixParam(private_nh);
  ROS_DEBUG_STREAM("tf_prefix: " << tf_prefix);
  config_.frame_id = tf::resolve(tf_prefix, config_.frame_id);

  // get model name, validate string, determine packet rate
  private_nh.param("model", config_.model, std::string("64E"));
  double packet_rate;                   // packet frequency (Hz)
  std::string model_full_name;
  if ((config_.model == "64E_S2") || 
      (config_.model == "64E_S2.1"))    // generates 1333312 points per second
    {                                   // 1 packet holds 384 points
//      packet_rate = 3472.17;            // 1333312 / 384
      packet_rate = 5787;
      model_full_name = std::string("HDL-") + config_.model;
    }
//    TODO
//  else if (config_.model == "64E_S3")
//    {
//      packet_rate = 5787;
//      model_full_name = std::string("HDL-") + config_.model;
//    }
  else if (config_.model == "64E")
    {
      packet_rate = 2600.0;
      model_full_name = std::string("HDL-") + config_.model;
    }
  else if (config_.model == "32E")
    {
      packet_rate = 1808.0;
      model_full_name = std::string("HDL-") + config_.model;
    }
  else if (config_.model == "VLP16")
    {
      packet_rate = 754;             // 754 Packets/Second for Last or Strongest mode 1508 for dual (VLP-16 User Manual)
      model_full_name = "VLP-16";
    }
  else
    {
      ROS_ERROR_STREAM("unknown Velodyne LIDAR model: " << config_.model);
      packet_rate = 2600.0;
    }
  std::string deviceName(std::string("Velodyne ") + model_full_name);

  private_nh.param("rpm", config_.rpm, 600.0);
  ROS_INFO_STREAM(deviceName << " rotating at " << config_.rpm << " RPM");
  double frequency = (config_.rpm / 60.0);     // expected Hz rate
  std::cout << "frequency: " << frequency << std::endl;
  // default number of packets for each scan is a single revolution
  // (fractions rounded up)
  config_.npackets = (int) ceil(packet_rate / frequency);
  private_nh.getParam("npackets", config_.npackets);
  ROS_INFO_STREAM("publishing " << config_.npackets << " packets per scan");

  std::string dump_file;
  private_nh.param("pcap", dump_file, std::string(""));

  int udp_port;
  private_nh.param("port", udp_port, (int) DATA_PORT_NUMBER);

  // Initialize dynamic reconfigure
  srv_ = boost::make_shared <dynamic_reconfigure::Server<velodyne_driver::
    VelodyneNodeConfig> > (private_nh);
  dynamic_reconfigure::Server<velodyne_driver::VelodyneNodeConfig>::
    CallbackType f;
  f = boost::bind (&VelodyneDriver::callback, this, _1, _2);
  srv_->setCallback (f); // Set callback function und call initially

  // initialize diagnostics
  diagnostics_.setHardwareID(deviceName);
  const double diag_freq = packet_rate/config_.npackets;
  diag_max_freq_ = diag_freq;
  diag_min_freq_ = diag_freq;
  ROS_INFO("expected frequency: %.3f (Hz)", diag_freq);

  using namespace diagnostic_updater;
  diag_topic_.reset(new TopicDiagnostic("velodyne_packets", diagnostics_,
                                        FrequencyStatusParam(&diag_min_freq_,
                                                             &diag_max_freq_,
                                                             0.1, 10),
                                        TimeStampStatusParam()));

  // open Velodyne input device or file
  if (dump_file != "")                  // have PCAP file?
    {
      // read data from packet capture file
      input_.reset(new velodyne_driver::InputPCAP(private_nh, udp_port,
                                                  packet_rate, dump_file));
    }
  else
    {
      // read data from live socket
      input_.reset(new velodyne_driver::InputSocket(private_nh, udp_port));
    }

  // raw packet output topic
  output_ =
    node.advertise<velodyne_msgs::VelodyneScan>("velodyne_packets", 10);

  t_output_ = node.advertise<std_msgs::Int8>("trigger", 10);
  t_input_ = node.subscribe("camera2/trigger_ret", 10, &VelodyneDriver::trigger_callback, this);
}

/** poll the device
 *
 *  @returns true unless end of file reached
 */
bool VelodyneDriver::poll(void)
{
  using namespace my_velodyne_rawdata;
         
  // Allocate a new shared pointer for zero-copy sharing with other nodelets.
  velodyne_msgs::VelodyneScanPtr scan(new velodyne_msgs::VelodyneScan);
  scan->packets.resize(config_.npackets);

  // Since the velodyne delivers data at a very high rate, keep
  // reading and publishing scans as fast as possible.

  //velodyne trigger at 0 degree, collect data at 270 degree
  //author: Zhibei Ma
  int i = 0;
  bool flag = false;
  double rotation = 1000;
  double before = 1000;
  int pkt_year, pkt_month, pkt_date, pkt_hours, pkt_minutes, pkt_seconds, pkt_gps_status;
  double rotation_gate = 270;
  double unit_angle = 360.0/config_.npackets;


  while (i < config_.npackets)
  {
    before = rotation;
    while (true) {
      // keep reading until full packet received
      int rc = input_->getPacket(&scan->packets[i], config_.time_offset);
      const raw_packet_t *raw = (const raw_packet_t *) &scan->packets[i].data[0];
      rotation =  raw->blocks[0].rotation/100.0;
      //if (rc ==0) 
      //  unixTime(&scan->packets[i],pkt_year, pkt_month, pkt_date, pkt_hours, pkt_minutes, pkt_seconds, pkt_gps_status);
      // if (rc == 0) (scan->packets[i]).stamp = VelodyneDriver::unixTime(&scan->packets[i]);        
      if (rc == 0) break;       // got a full packet?
      if (rc < 0) return false; // end of file reached?
    }
    ROS_INFO("%d %f %f ", i, rotation, rotation - before);

    // trigger
    if (rotation >= 0 && rotation <= unit_angle && i > 1) {
      std_msgs::Int8 msg;
      msg.data = 1;
      t_output_.publish(msg);
    }

    // store velodyne packets
    if (rotation >= rotation_gate && rotation <= rotation_gate + unit_angle && i > 1)
       break;
    // doesn't need to keep frequency at 20hz
    i++;
  }
  
  ROS_DEBUG("Publishing a full Velodyne scan.");
  scan->header.stamp = scan->packets[72].stamp;  //174*90/360=43, 5787/20 // npackets*90/360=72   
  scan->header.frame_id = config_.frame_id;

  // notify diagnostics that a message has been published, updating
  // its status
  std::cout<<"stamp 1:"<<scan->header.stamp<<std::endl;
  diag_topic_->tick(scan->header.stamp);
  diagnostics_.update();

  //filter 0 message
  if (scan->header.stamp.toSec()) {
      output_.publish(scan);
      std::cout<<"stamp 1:"<<scan->header.stamp<<std::endl;
  }
  return true;
}



void VelodyneDriver::callback(velodyne_driver::VelodyneNodeConfig &config,
              uint32_t level)
{
  ROS_INFO("Reconfigure Request");
  config_.time_offset = config.time_offset;
}


void VelodyneDriver::trigger_callback(const std_msgs::Int8::ConstPtr &msg){
  std::cout<<"lidar subscri count: " <<int(msg -> data)<<" "<<ros::WallTime::now()<<" "<<ros::Time::now()<<std::endl;
}


void VelodyneDriver::unixTime(velodyne_msgs::VelodynePacket *pkt, int& pkt_year, int& pkt_month, int& pkt_date,
                              int& pkt_hours, int& pkt_minutes, int& pkt_seconds, int& pkt_gps_status){
    //pkt = &scan->packets[i]
    // int pkt_hours, pkt_minutes, pkt_seconds, pkt_date, pkt_month, pkt_year, pkt_gps_status;


    int status_type = pkt->data[1204];
    int status_value = pkt->data[1205];
    int gps_time_0 = pkt->data[1200];
    int gps_time_1 = pkt->data[1201];
    int gps_time_2 = pkt->data[1202];
    int gps_time_3 = pkt->data[1203];
          
    // for test
    // status_type = 71;

    switch (status_type){
        case 72: pkt_hours = status_value; 
        case 77: pkt_minutes = status_value;
        case 83: pkt_seconds = status_value;
        case 68: pkt_date = status_value;
        case 78: pkt_month = status_value;
        case 89: pkt_year = status_value;
        case 71: pkt_gps_status = status_value;
        default: break;
    }

    //transform year/month/date/hour/minute/second to unix time
    //get gps time
    uint32_t gps_time; 
    double gps_secs;
    gps_time = gps_time_0 + gps_time_1*256 + gps_time_2*256*256 + gps_time_3*256*256*256; 
    gps_secs = gps_time/1000000;
    std::cout<<"gps_secs: "<<gps_secs<<std::endl;

    time_t rawtime;
    struct tm * timeinfo;
    timeinfo = localtime(&rawtime);
    timeinfo->tm_year = pkt_year-1900;
    timeinfo->tm_mon = pkt_month - 1;
    timeinfo->tm_mday = pkt_date;
    timeinfo->tm_hour = pkt_hours;
    timeinfo->tm_min = pkt_minutes;
    timeinfo->tm_sec = 30.32323;
    std::cout<<mktime(timeinfo)<<std::endl;;
          
    double cur_time = mktime(timeinfo) + pkt_seconds - int(pkt_seconds); 
    // TODO 
    // pkt->stamp = ros::Time(cur_time);

    // std::cout<<cur_time<<std::endl;
    // return cur_time;
}

} // namespace velodyne_driver
