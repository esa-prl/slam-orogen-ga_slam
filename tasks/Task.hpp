#ifndef _GASLAM_TASK_HPP_
#define _GASLAM_TASK_HPP_

#include "ga_slam/TaskBase.hpp"
#include "ga_slam/GaSlam.hpp"

namespace ga_slam {

class Task : public TaskBase {
  friend class TaskBase;

  public:
    Task(std::string const& name = "ga_slam::Task",
        TaskCore::TaskState initial_state = Stopped)
        : TaskBase(name, initial_state) {}

    Task(std::string const& name,
        RTT::ExecutionEngine* engine,
        TaskCore::TaskState initial_state = Stopped)
        : TaskBase(name, engine, initial_state) {}

    virtual ~Task() {}

    bool configureHook() { return TaskBase::configureHook(); }
};

}  // namespace ga_slam

#endif  // _GASLAM_TASK_HPP_

