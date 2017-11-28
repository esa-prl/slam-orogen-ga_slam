#include "Task.hpp"

namespace ga_slam {

Task::Task(std::string const& name)
        : TaskBase(name) {
    gaSlam_.reset(new GaSlam());
    inputPCLCloud_.reset(new PointCloud);
}

void Task::pointCloudTransformerCallback(
        const base::Time& timestamp,
        const base::samples::Pointcloud& inputBaseCloud) {
    if (!readPoseAndTF(timestamp))
        return;

    inputPose_ = inputBasePose_.getTransform();
    convertBaseToPCL(inputBaseCloud, inputPCLCloud_);

    gaSlam_->registerData(inputPose_, cameraToMapTF_, inputPCLCloud_);

    if (_debugEnabled.rvalue()) {
        convertPCLToBase(filteredBaseCloud_, gaSlam_->getFilteredPointCloud());
        _filteredPointCloud.write(filteredBaseCloud_);
    }
}

bool Task::readPoseAndTF(const base::Time& timestamp) {
    if (!_pose.connected()) {
        RTT::log(RTT::Error) << "[GA SLAM] Input pose port is not connected."
                <<  RTT::endlog();
        error(INPUT_NOT_CONNECTED);
        return false;
    }

    if (_pose.readNewest(inputBasePose_) != RTT::NewData) {
        RTT::log(RTT::Warning) << "[GA SLAM] Cannot associate the input point"
                << "cloud to the newest input pose." << std::endl;
        report(INPUTS_NOT_ALIGNED);
        return false;
    } else {
        state(RUNNING);
    }

    if (!_slamCamera2body.get(timestamp, cameraToMapTF_, false)) {
        RTT::log(RTT::Error) << "[GA SLAM] Camera to body TF not found."
                << RTT::endlog();
        error(TRANSFORM_NOT_FOUND);
        return false;
    }

    return true;
}

void Task::convertBaseToPCL(
        const base::samples::Pointcloud& baseCloud,
        PointCloud::Ptr& pclCloud) {
    pclCloud->clear();
    pclCloud->is_dense = true;
    pclCloud->header.stamp = baseCloud.time.toMicroseconds();

    for (auto& point : baseCloud.points)
        pclCloud->push_back(pcl::PointXYZ(point.x(), point.y(), point.z()));
}

void Task::convertPCLToBase(
        base::samples::Pointcloud& baseCloud,
        const PointCloud::ConstPtr& pclCloud) {
    baseCloud.points.clear();
    baseCloud.time.fromMicroseconds(pclCloud->header.stamp);

    for (auto& point : pclCloud->points)
        baseCloud.points.push_back(base::Point(point.x, point.y, point.z));
}

}  // namespace ga_slam

