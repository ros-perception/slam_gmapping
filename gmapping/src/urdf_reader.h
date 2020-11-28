#include <vector>
#include <string>
#include "tf/transform_listener.h"

std::vector<geometry_msgs::TransformStamped> ReadStaticTransformsFromUrdf(
    const std::string& urdf_filename);
