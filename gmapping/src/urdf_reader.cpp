#include "urdf_reader.h"

#include <string>
#include <vector>
#include "urdf/model.h"

std::vector<geometry_msgs::TransformStamped> ReadStaticTransformsFromUrdf(
    const std::string& urdf_filename) {
  urdf::Model model;
  model.initFile(urdf_filename);
#if URDFDOM_HEADERS_HAS_SHARED_PTR_DEFS
  std::vector<urdf::LinkSharedPtr> links;
#else
  std::vector<boost::shared_ptr<urdf::Link> > links;
#endif
  model.getLinks(links);
  std::vector<geometry_msgs::TransformStamped> transforms;
  for (const auto& link : links) {
    if (!link->getParent() || link->parent_joint->type != urdf::Joint::FIXED) {
      continue;
    }
    const urdf::Pose& pose =
        link->parent_joint->parent_to_joint_origin_transform;
    geometry_msgs::TransformStamped transform;
    transform.transform.translation.x = pose.position.x;
    transform.transform.translation.y = pose.position.y;
    transform.transform.translation.z = pose.position.z;
    transform.transform.rotation.x = pose.rotation.x;
    transform.transform.rotation.y = pose.rotation.y;
    transform.transform.rotation.z = pose.rotation.z;
    transform.transform.rotation.w = pose.rotation.w;
    transform.child_frame_id = link->name;
    transform.header.frame_id = link->getParent()->name;
    transform.header.stamp = ros::Time(0);
    transforms.push_back(transform);
  }
  return transforms;
}
