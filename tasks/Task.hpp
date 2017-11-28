#ifndef _GASLAM_TASK_HPP_
#define _GASLAM_TASK_HPP_

#include "ga_slam/TaskBase.hpp"
#include "ga_slam/GaSlam.hpp"

namespace ga_slam {

class Task : public TaskBase {
  friend class TaskBase;

  public:
    explicit Task(std::string const& name = "ga_slam::Task");

  protected:
    void pointCloudTransformerCallback(
            const base::Time& timestamp,
            const base::samples::Pointcloud& inputBaseCloud);

    bool readPoseAndTF(const base::Time& timestamp);

    static void convertBaseToPCL(
            const base::samples::Pointcloud& baseCloud,
            PointCloud::Ptr& pclCloud);

    static void convertPCLToBase(
            base::samples::Pointcloud& baseCloud,
            const PointCloud::ConstPtr& pclCloud);

  protected:

    base::samples::RigidBodyState inputBasePose_;

    Pose cameraToMapTF_;
};

}  // namespace ga_slam

#endif  // _GASLAM_TASK_HPP_

