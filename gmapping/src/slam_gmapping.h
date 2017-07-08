/*
 * slam_gmapping
 * Copyright (c) 2017, Open Source Robotics Foundation, Inc.
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

/* Author: Brian Gerkey */

/* rclcpp */
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>
#include <rcutils/cmdline_parser.h>

/* sensor messages */
#include <sensor_msgs/msg/laser_scan.hpp>

/* standard messages */
#include <std_msgs/msg/float64.hpp>

/* navigation messages and services */
#include <nav_msgs/msg/map_meta_data.hpp>
#include <nav_msgs/srv/get_map.hpp>

/* tf2_ros */
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>

/* ??? */
/* #include <tf2_ros/message_filter.h> */
/* #include "message_filters/subscriber.h" */

/* OpenSLAM GMapping */
#include <gmapping/gridfastslam/gridslamprocessor.h>
#include <gmapping/sensor/sensor_base/sensor.h>
#include <gmapping/sensor/sensor_range/rangesensor.h>
#include <gmapping/sensor/sensor_odometry/odometrysensor.h>

/* STL includes */
#include <functional>
#include <iostream>
#include <memory>
#include <thread>
#include <mutex>
#include <ctime>

/* hacky fix for lack of these macros */
#define ROS_ASSERT assert
#define ROS_DEBUG printf
#define ROS_WARN printf
#define ROS_INFO printf

class SlamGMapping
{
public:
  SlamGMapping(std::shared_ptr<rclcpp::Node> _node);
  /* SlamGMapping(ros::NodeHandle& nh, ros::NodeHandle& pnh); */
  /* SlamGMapping(unsigned long int seed, unsigned long int max_duration_buffer); */
  ~SlamGMapping();

  void init();
  void startLiveSlam();
  void startReplay(const std::string & bag_fname, std::string scan_topic);
  void publishTransform();

  void laserCallback(const std::shared_ptr<sensor_msgs::msg::LaserScan> & scan);
  bool mapCallback(const std::shared_ptr<nav_msgs::srv::GetMap::Request> req,
                   std::shared_ptr<nav_msgs::srv::GetMap::Response> res);
  void publishLoop(double transform_publish_period);

private:
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr entropy_publisher_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr sst_;
  rclcpp::Publisher<nav_msgs::msg::MapMetaData>::SharedPtr sstm_;
  rclcpp::service::ServiceBase::SharedPtr ss_;
  tf2_ros::Buffer * buffer;
  tf2_ros::TransformListener * tf_;
  /* message_filters::Subscriber<sensor_msgs::LaserScan>* scan_filter_sub_; */
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_filter_sub_;
  /* tf2_ros::MessageFilter<sensor_msgs::msg::LaserScan> * scan_filter_; */
  tf2_ros::TransformBroadcaster * tfB_;

  GMapping::GridSlamProcessor* gsp_;
  GMapping::RangeSensor* gsp_laser_ = nullptr;
  // The angles in the laser, going from -x to x (adjustment is made to get the laser between
  // symmetrical bounds as that's what gmapping expects)
  std::vector<double> laser_angles_;
  // The pose, in the original laser frame, of the corresponding centered laser with z facing up
  tf2::Stamped<tf2::Transform> centered_laser_pose_;
  // Depending on the order of the elements in the scan and the orientation of the scan frame,
  // We might need to change the order of the scan
  bool do_reverse_range_;
  unsigned int gsp_laser_beam_count_;
  GMapping::OdometrySensor* gsp_odom_ = nullptr;

  bool got_first_scan_ = false;
  bool got_map_ = false;

  nav_msgs::srv::GetMap::Response map_;

  tf2::Duration map_update_interval_;
  tf2::Transform map_to_odom_;
  std::mutex map_to_odom_mutex_;
  std::mutex map_mutex_;

  unsigned int laser_count_ = 0;
  unsigned int throttle_scans_;

  std::thread * transform_thread_ = nullptr;

  std::string base_frame_;
  std::string laser_frame_;
  std::string map_frame_;
  std::string odom_frame_;

  void updateMap(const std::shared_ptr<sensor_msgs::msg::LaserScan> scan);
  bool getOdomPose(GMapping::OrientedPoint& gmap_pose, const rclcpp::Time & t);
  bool initMapper(const std::shared_ptr<sensor_msgs::msg::LaserScan> scan);
  bool addScan(const std::shared_ptr<sensor_msgs::msg::LaserScan> scan, GMapping::OrientedPoint & gmap_pose);
  double computePoseEntropy();

  // Parameters used by GMapping
  double maxRange_;
  double maxUrange_;
  double maxrange_;
  double minimum_score_;
  double sigma_;
  int kernelSize_;
  double lstep_;
  double astep_;
  int iterations_;
  double lsigma_;
  double ogain_;
  int lskip_;
  double srr_;
  double srt_;
  double str_;
  double stt_;
  double linearUpdate_;
  double angularUpdate_;
  double temporalUpdate_;
  double resampleThreshold_;
  int particles_;
  double xmin_;
  double ymin_;
  double xmax_;
  double ymax_;
  double delta_;
  double occ_thresh_;
  double llsamplerange_;
  double llsamplestep_;
  double lasamplerange_;
  double lasamplestep_;

  std::shared_ptr<rclcpp::Node> node;

  unsigned long int seed_ = time(nullptr);

  double transform_publish_period_;
  double tf_delay_;
};
