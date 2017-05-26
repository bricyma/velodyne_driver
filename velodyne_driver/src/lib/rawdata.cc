/*
 *  Copyright (C) 2007 Austin Robot Technology, Patrick Beeson
 *  Copyright (C) 2009, 2010, 2012 Austin Robot Technology, Jack O'Quin
 *
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

/**
 *  @file
 *
 *  Velodyne 3D LIDAR data accessor class implementation.
 *
 *  Class for unpacking raw Velodyne LIDAR packets into useful
 *  formats.
 *
 *  Derived classes accept raw Velodyne data for either single packets
 *  or entire rotations, and provide it in various formats for either
 *  on-line or off-line processing.
 *
 *  @author Patrick Beeson
 *  @author Jack O'Quin
 *
 *  HDL-64E S2 calibration support provided by Nick Hillier
 */

#include <fstream>
#include <math.h>

#include <ros/ros.h>
#include <ros/package.h>
#include <angles/angles.h>

#include <velodyne_driver/rawdata.h>

namespace my_velodyne_rawdata
{
  ////////////////////////////////////////////////////////////////////////
  //
  // RawData base class implementation
  //
  ////////////////////////////////////////////////////////////////////////

  RawData::RawData() {}
  



  /*
    return packet's rotation
   */
  int RawData::unpack(const velodyne_msgs::VelodynePacket &pkt)
  {

    std::cout<<"in the unpack **********************"<<std::endl;
    ROS_DEBUG_STREAM("Received packet, time: " << pkt.stamp);
    

    const raw_packet_t *raw = (const raw_packet_t *) &pkt.data[0];

    return raw->blocks[0].rotation/100;
  
  }
}
  

 // namespace my_velodyne_rawdata
