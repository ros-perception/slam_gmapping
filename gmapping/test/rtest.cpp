/*
 * Copyright (c) 2008, Willow Garage, Inc.
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

/* Author: Brian Gerkey */

#include <gtest/gtest.h>
#include <ros/service.h>
#include <ros/ros.h>
#include <nav_msgs/GetMap.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/MapMetaData.h>


ros::NodeHandle* g_n=NULL;
double g_res, g_width, g_height, g_min_free_ratio, g_max_free_ratio;

class MapClientTest : public testing::Test
{
  public:
    MapClientTest()
    {
      got_map_ = false;
      got_map_metadata_ = false;
    }

    ~MapClientTest()
    {
    }

    bool got_map_;
    boost::shared_ptr<nav_msgs::OccupancyGrid const> map_;
    void mapCallback(const boost::shared_ptr<nav_msgs::OccupancyGrid const>& map)
    {
      map_ = map;
      got_map_ = true;
    }

    bool got_map_metadata_;
    boost::shared_ptr<nav_msgs::MapMetaData const> map_metadata_;
    void mapMetaDataCallback(const boost::shared_ptr<nav_msgs::MapMetaData const>& map_metadata)
    {
      map_metadata_ = map_metadata;
      got_map_metadata_ = true;
    }

    void checkMapMetaData(const nav_msgs::MapMetaData& map_metadata)
    {
      EXPECT_FLOAT_EQ(map_metadata.resolution, g_res);
      EXPECT_FLOAT_EQ(map_metadata.width, g_width);
      EXPECT_FLOAT_EQ(map_metadata.height, g_height);
    }

    void checkMapData(const nav_msgs::OccupancyGrid& map)
    {
      unsigned int i;
      unsigned int free_cnt = 0;
      for(i=0; i < map.info.width * map.info.height; i++)
      {
        if(map.data[i] == 0)
          free_cnt++;
      }
      double free_ratio = free_cnt / (double)(i);
      EXPECT_GE(free_ratio, g_min_free_ratio);
      EXPECT_LE(free_ratio, g_max_free_ratio);
    }
};

/* Try to retrieve the map via service, and compare to ground truth */
TEST_F(MapClientTest, call_service)
{
  nav_msgs::GetMap::Request  req;
  nav_msgs::GetMap::Response resp;
  ASSERT_TRUE(ros::service::waitForService("dynamic_map", 5000));
  ASSERT_TRUE(ros::service::call("dynamic_map", req, resp));
  checkMapMetaData(resp.map.info);
  checkMapData(resp.map);
}

/* Try to retrieve the map via topic, and compare to ground truth */
TEST_F(MapClientTest, subscribe_topic)
{
  ros::Subscriber sub = g_n->subscribe<nav_msgs::OccupancyGrid>("map", 1, boost::bind(&MapClientTest::mapCallback, this, _1));

  // Try a few times, because the server may not be up yet.
  int i=20;
  while(!got_map_ && i > 0)
  {
    ros::spinOnce();
    ros::WallDuration d(0.25);
    d.sleep();
    i--;
  }
  ASSERT_TRUE(got_map_);
  checkMapMetaData(map_->info);
  checkMapData(*(map_.get()));
}

/* Try to retrieve the metadata via topic, and compare to ground truth */
TEST_F(MapClientTest, subscribe_topic_metadata)
{
  ros::Subscriber sub = g_n->subscribe<nav_msgs::MapMetaData>("map_metadata", 1, boost::bind(&MapClientTest::mapMetaDataCallback, this, _1));

  // Try a few times, because the server may not be up yet.
  int i=20;
  while(!got_map_metadata_ && i > 0)
  {
    ros::spinOnce();
    ros::WallDuration d(0.25);
    d.sleep();
    i--;
  }
  ASSERT_TRUE(got_map_metadata_);
  checkMapMetaData(*(map_metadata_.get()));
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);

  ros::init(argc, argv, "map_client_test");
  g_n = new ros::NodeHandle();

  // Required args are, in order:
  //   <delay> <resolution> <width> <height> <min_free_ratio> <max_free_ratio>
  ROS_ASSERT(argc == 7);
  ros::Duration delay = ros::Duration(atof(argv[1]));
  g_res = atof(argv[2]);
  g_width = atof(argv[3]);
  g_height = atof(argv[4]);
  g_min_free_ratio = atof(argv[5]);
  g_max_free_ratio = atof(argv[6]);

  while(ros::Time::now().toSec() == 0.0)
  {
    ROS_INFO("Waiting for initial time publication");
    ros::Duration(0.25).sleep();
    ros::spinOnce();
  }
  ros::Time start_time = ros::Time::now();
  while((ros::Time::now() - start_time) < delay)
  {
    ROS_INFO("Waiting for delay expiration (%.3f - %.3f) < %.3f",
             ros::Time::now().toSec(),
             start_time.toSec(),
             delay.toSec());
    ros::Duration(0.25).sleep();
    ros::spinOnce();
  }

  int result = RUN_ALL_TESTS();
  delete g_n;
  return result;
}
