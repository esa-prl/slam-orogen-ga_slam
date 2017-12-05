#ifndef _GASLAM_TASK_HPP_
#define _GASLAM_TASK_HPP_

#include "ga_slam/TaskBase.hpp"
#include "ga_slam/GaSlam.hpp"

namespace ga_slam {

class Task : public TaskBase {
  friend class TaskBase;

  public:
    explicit Task(std::string const& name = "ga_slam::Task");

    bool configureHook(void) override;

  protected:
    void pointCloudTransformerCallback(
            const base::Time& timestamp,
            const base::samples::Pointcloud& inputBaseCloud) override;

    bool readPoseAndTF(const base::Time& timestamp);

    static void convertBaseCloudToPCL(
            const base::samples::Pointcloud& baseCloud,
            PointCloud::Ptr& pclCloud);

    static void convertPCLToBaseCloud(
            base::samples::Pointcloud& baseCloud,
            const PointCloud::ConstPtr& pclCloud);

    static void convertBaseFrameToMap(
        const base::samples::frame::Frame& frame,
        Map& map,
        const double& resolution,
        const double& positionX,
        const double& positionY,
        const double& minElevation,
        const double& maxElevation);

    static void convertMapToBaseFrame(
        base::samples::frame::Frame& frame,
        const Map& map,
        const double& minElevation,
        const double& maxElevation);

    static void convertMapToBaseCloud(
        base::samples::Pointcloud& baseCloud,
        const Map& map);

  protected:
    GaSlam gaSlam_;

    PointCloud::Ptr inputPCLCloud_;
    base::samples::Pointcloud filteredBaseCloud_;
    base::samples::Pointcloud mapBaseCloud_;

    base::samples::frame::Frame rawMapBaseFrame_;

    Pose inputPose_;
    base::samples::RigidBodyState inputBasePose_;

    Pose cameraToMapTF_;
};

}  // namespace ga_slam

#endif  // _GASLAM_TASK_HPP_

