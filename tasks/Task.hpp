#pragma once

#include "ga_slam/TaskBase.hpp"
#include "ga_slam/GaSlam.hpp"

#include <chrono>
#include <future>

namespace ga_slam {

using BaseTime = base::Time;
using BaseCloud = base::samples::Pointcloud;
using BaseImage = base::samples::DistanceImage;
using BasePose = base::samples::RigidBodyState;

class Task : public TaskBase {
  public:
    explicit Task(std::string const& name = "ga_slam::Task")
            : TaskBase(name),
              gaSlam_() {}

  protected:
    bool configureHook(void) override;

    void updateHook(void) override;

    void cloudCallback(
        const BaseCloud& baseCloud,
        const Pose& sensorToBodyTF);

    void outputDebugInfo(void);

    template<typename T>
    bool isFutureReady(const std::future<T>& future) {
        if (!future.valid()) return true;
        return (future.wait_for(std::chrono::milliseconds(0)) ==
                std::future_status::ready);
    }

  protected:
    GaSlam gaSlam_;

    Pose bodyToGroundTF_;
    Pose hazcamToBodyTF_;
    Pose loccamToBodyTF_;
    BasePose basePancamToBodyTF_;

    BasePose poseGuess_;
    BaseCloud hazcamCloud_;
    BaseCloud loccamCloud_;
    BaseCloud pancamCloud_;

    BasePose orbiterCloudPose_;
    BaseCloud orbiterCloud_;

    std::future<void> poseGuessFuture_;
    std::future<void> hazcamCloudFuture_;
    std::future<void> loccamCloudFuture_;
    std::future<void> pancamCloudFuture_;
    std::future<void> orbiterCloudFuture_;
};

}  // namespace ga_slam

