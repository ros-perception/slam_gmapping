/*
 * slam_gmapping
 * Copyright (c) 2008, Willow Garage, Inc.
 *
 * THE WORK (AS DEFINED BELOW) IS PROVIDED UNDER THE TERMS OF THIS CREATIVE
 * COMMONS PUBLIC LICENSE ("CCPL" OR "LICENSE"). THE WORK IS PROTECTED BY
 * COPYRIGHT AND/OR OTHER APPLICABLE LAW. ANY USE OF THE WORK OTHER THAN AS
 * AUTHORIZED UNDER THIS LICENSE OR COPYRIGHT LAW IS PROHIBITED.
 * 
 * BY EXERCISING ANY RIGHTS TO THE WORK PROVIDED HERE, YOU ACCEPT AND AGREE TO
 * BE BOUND BY THE TERMS OF THIS LICENSE. THE LICENSOR GRANTS YOU THE RIGHTS
 * CONTAINED HERE IN CONSIDERATION OF YOUR ACCEPTANCE OF SUCH TERMS AND
 * CONDITIONS.
 *
 */

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include "slam_gmapping.h"

class SlamGMappingNodelet : public nodelet::Nodelet
{
  public:
    SlamGMappingNodelet()  {}

    ~SlamGMappingNodelet() {}
  
    virtual void onInit()
    {
      NODELET_INFO_STREAM("Initialising Slam GMapping nodelet...");
      sg_.reset(new SlamGMapping(getNodeHandle(), getPrivateNodeHandle()));
      NODELET_INFO_STREAM("Starting live SLAM...");
      sg_->startLiveSlam();
    }

  private:  
    boost::shared_ptr<SlamGMapping> sg_;
};

PLUGINLIB_EXPORT_CLASS(SlamGMappingNodelet, nodelet::Nodelet)
