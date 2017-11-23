#include "Task.hpp"

namespace ga_slam {

void Task::inputPointCloudTransformerCallback(
        const base::Time& timestamp,
        const base::samples::Pointcloud& pointCloud) {}

void Task::inputPoseTransformerCallback(
        const base::Time& timestamp,
        const base::samples::RigidBodyState& pose) {}

}  // namespace ga_slam

