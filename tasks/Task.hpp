#ifndef _GASLAM_TASK_HPP_
#define _GASLAM_TASK_HPP_

#include "ga_slam/TaskBase.hpp"
#include "ga_slam/GaSlam.hpp"

namespace ga_slam {

class Task : public TaskBase {
  friend class TaskBase;

  public:
    Task(std::string const& name = "ga_slam::Task")
            : TaskBase(name) {}

    Task(std::string const& name, RTT::ExecutionEngine* engine)
            : TaskBase(name, engine) {}

  protected:
    void inputPointCloudTransformerCallback(
            const base::Time& timestamp,
            const base::samples::Pointcloud& pointCloud);

    void inputPoseTransformerCallback(
            const base::Time& timestamp,
            const base::samples::RigidBodyState& pose);
};

}  // namespace ga_slam

#endif  // _GASLAM_TASK_HPP_

