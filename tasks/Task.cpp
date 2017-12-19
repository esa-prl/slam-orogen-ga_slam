#include "Task.hpp"
#include "GaSlamBaseConverter.hpp"

#include "grid_map_cereal/GridMapCereal.hpp"

namespace ga_slam {

Task::Task(std::string const& name)
        : TaskBase(name),
          gaSlam_() {
    inputCloud_.reset(new Cloud);
}

bool Task::configureHook(void) {
    if (!TaskBase::configureHook()) return false;

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

void Task::cloudTransformerCallback(
        const BaseTime& timestamp,
        const BaseCloud& inputBaseCloud) {
    if (!readPoseAndTF(timestamp)) return;

    GaSlamBaseConverter::convertBaseCloudToPCL(inputBaseCloud, inputCloud_);

    gaSlam_.registerData(inputCloud_, inputPose_,
            sensorToBodyTF_, bodyToGroundTF_);

    if (_debugInfoEnabled.rvalue()) outputDebugInfo();
}

bool Task::readPoseAndTF(const BaseTime& timestamp) {
    if (!_slamSensor2body.get(timestamp, sensorToBodyTF_, false) ||
            !_body2ground.get(timestamp, bodyToGroundTF_, false)) {
        RTT::log(RTT::Error) << "[GA SLAM] Sensor to body and body to map"
                << " transformations not found." << RTT::endlog();
        error(TRANSFORM_NOT_FOUND);
        return false;
    }

    if (!_pose.connected()) {
        RTT::log(RTT::Error) << "[GA SLAM] Input pose port is not connected."
                <<  RTT::endlog();
        error(INPUT_NOT_CONNECTED);
        return false;
    }

    BasePose inputBasePose;
    if (_pose.readNewest(inputBasePose) != RTT::NewData) {
        RTT::log(RTT::Warning) << "[GA SLAM] Cannot associate the input point"
                << " cloud to the newest input pose." << std::endl;
        report(INPUTS_NOT_ALIGNED);
        return false;
    } else {
        inputPose_ = inputBasePose.getTransform();

        state(RUNNING);
    }

    return true;
}

void Task::outputDebugInfo(void) {
    if (_rawMapDebugEnabled.rvalue()) {
        BaseImage rawMapBaseImage;

        GaSlamBaseConverter::convertMapToBaseImage(rawMapBaseImage,
                gaSlam_.getRawMap());

        _rawElevationMap.write(rawMapBaseImage);
    }

    if (_serializationDebugEnabled.rvalue()) {
        saveGridMap(gaSlam_.getRawMap(), _saveMapPath.rvalue());
        savePose(gaSlam_.getCorrectedPose(), _savePosePath.rvalue());
    }

    if (_cloudDebugEnabled.rvalue()) {
        BaseCloud filteredBaseCloud, mapBaseCloud;

        GaSlamBaseConverter::convertPCLToBaseCloud(filteredBaseCloud,
                gaSlam_.getFilteredCloud());
        GaSlamBaseConverter::convertMapToBaseCloud(mapBaseCloud,
                gaSlam_.getRawMap());

        _filteredCloud.write(filteredBaseCloud);
        _mapCloud.write(mapBaseCloud);
    }
}

}  // namespace ga_slam

