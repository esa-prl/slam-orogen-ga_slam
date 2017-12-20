#pragma once

#include "ga_slam/TaskBase.hpp"
#include "ga_slam/GaSlam.hpp"

namespace ga_slam {

using BaseTime = base::Time;
using BaseCloud = base::samples::Pointcloud;
using BaseImage = base::samples::DistanceImage;
using BasePose = base::samples::RigidBodyState;

class Task : public TaskBase {
  public:
    explicit Task(std::string const& name = "ga_slam::Task");

    bool configureHook(void) override;

  protected:
    void cloudTransformerCallback(
            const BaseTime& timestamp,
            const BaseCloud& baseCloud) override;

    bool readPoseAndTF(const BaseTime& timestamp);

    void outputDebugInfo(void);

  protected:
    GaSlam gaSlam_;

    Cloud::Ptr cloud_;

    Pose poseGuess_;

    Pose sensorToBodyTF_;
    Pose bodyToGroundTF_;
};

}  // namespace ga_slam

