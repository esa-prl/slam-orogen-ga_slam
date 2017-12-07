#include "Task.hpp"
#include "GaSlamBaseConverter.hpp"

#include "grid_map_cereal/GridMapCereal.hpp"

namespace ga_slam {

Task::Task(std::string const& name)
        : TaskBase(name),
          gaSlam_() {
    inputPCLCloud_.reset(new PointCloud);
}

bool Task::configureHook(void) {
    if (!TaskBase::configureHook())
        return false;

    if (!gaSlam_.setParameters(
                _mapSizeX.rvalue(), _mapSizeY.rvalue(),
                _robotPositionX.rvalue(), _robotPositionY.rvalue(),
                _mapResolution.rvalue(), _voxelSize.rvalue(),
                _minElevation.rvalue(), _maxElevation.rvalue())) {
        RTT::log(RTT::Error) << "[GA SLAM] Encountered error when"
                << " setting parameters." <<  RTT::endlog();
        error(PARAMETERS_NOT_SET);
        return false;
    }

    return true;
}

void Task::pointCloudTransformerCallback(
        const BaseTime& timestamp,
        const BaseCloud& inputBaseCloud) {
    if (!readPoseAndTF(timestamp))
        return;

    GaSlamBaseConverter::convertBaseCloudToPCL(inputBaseCloud, inputPCLCloud_);

    gaSlam_.registerData(inputPose_, cameraToMapTF_, inputPCLCloud_);

    if (_debugInfoEnabled.rvalue())
        outputDebugInfo();
}

bool Task::readPoseAndTF(const BaseTime& timestamp) {
    if (!_pose.connected()) {
        RTT::log(RTT::Error) << "[GA SLAM] Input pose port is not connected."
                <<  RTT::endlog();
        error(INPUT_NOT_CONNECTED);
        return false;
    }

    if (_pose.readNewest(inputBasePose_) != RTT::NewData) {
        RTT::log(RTT::Warning) << "[GA SLAM] Cannot associate the input point"
                << " cloud to the newest input pose." << std::endl;
        report(INPUTS_NOT_ALIGNED);
        return false;
    } else {
        inputPose_ = inputBasePose_.getTransform();

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

void Task::outputDebugInfo(void) {
    if (_rawMapDebugEnabled.rvalue()) {
        GaSlamBaseConverter::convertMapToBaseImage(rawMapBaseImage_,
                gaSlam_.getRawMap());

        _rawElevationMap.write(rawMapBaseImage_);
    }

    if (_serializationDebugEnabled.rvalue())
        saveGridMap(gaSlam_.getRawMap(), _savePath.rvalue());

    if (_pointCloudDebugEnabled.rvalue()) {
        GaSlamBaseConverter::convertPCLToBaseCloud(filteredBaseCloud_,
                gaSlam_.getFilteredPointCloud());
        GaSlamBaseConverter::convertMapToBaseCloud(mapBaseCloud_,
                gaSlam_.getRawMap());

        _filteredPointCloud.write(filteredBaseCloud_);
        _mapPointCloud.write(mapBaseCloud_);
    }
}

}  // namespace ga_slam

