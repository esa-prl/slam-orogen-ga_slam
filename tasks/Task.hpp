#ifndef _GASLAM_TASK_HPP_
#define _GASLAM_TASK_HPP_

#include "ga_slam/TaskBase.hpp"
#include "ga_slam/GaSlam.hpp"

using BaseTime = base::Time;
using BaseCloud = base::samples::Pointcloud;
using BaseImage = base::samples::DistanceImage;
using BasePose = base::samples::RigidBodyState;

namespace ga_slam {

class Task : public TaskBase {
  friend class TaskBase;

  public:
    explicit Task(std::string const& name = "ga_slam::Task");

    bool configureHook(void) override;

  protected:
    void pointCloudTransformerCallback(
            const BaseTime& timestamp,
            const BaseCloud& inputBaseCloud) override;

    bool readPoseAndTF(const BaseTime& timestamp);

    void outputDebugInfo(void);

  protected:
    GaSlam gaSlam_;

    PointCloud::Ptr inputPCLCloud_;
    BaseCloud filteredBaseCloud_;
    BaseCloud mapBaseCloud_;

    BaseImage rawMapBaseImage_;

    Pose inputPose_;
    BasePose inputBasePose_;

    Pose cameraToMapTF_;
};

}  // namespace ga_slam

#endif  // _GASLAM_TASK_HPP_

