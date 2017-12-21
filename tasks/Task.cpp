#include "Task.hpp"
#include "GaSlamBaseConverter.hpp"

#include "grid_map_cereal/GridMapCereal.hpp"

namespace ga_slam {

Task::Task(std::string const& name)
        : TaskBase(name),
          gaSlam_() {
    cloud_.reset(new Cloud);
}

bool Task::configureHook(void) {
    if (!TaskBase::configureHook()) return false;

    gaSlam_.setParameters(_mapSizeX.rvalue(), _mapSizeY.rvalue(),
            _robotPositionX.rvalue(), _robotPositionY.rvalue(),
            _mapResolution.rvalue(), _voxelSize.rvalue(),
            _minElevation.rvalue(), _maxElevation.rvalue());

    return true;
}

void Task::cloudTransformerCallback(
        const BaseTime& timestamp,
        const BaseCloud& baseCloud) {
    if (!readPoseAndTF(timestamp)) return;

    GaSlamBaseConverter::convertBaseCloudToPCL(baseCloud, cloud_);

    gaSlam_.cloudCallback(cloud_, sensorToBodyTF_, bodyToGroundTF_, poseGuess_);

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

    if (!_poseGuess.connected()) {
        RTT::log(RTT::Error) << "[GA SLAM] Input pose port is not connected."
                <<  RTT::endlog();
        error(INPUT_NOT_CONNECTED);

        return false;
    }

    BasePose basePoseGuess;
    if (_poseGuess.readNewest(basePoseGuess) != RTT::NewData) {
        RTT::log(RTT::Warning) << "[GA SLAM] Cannot associate the input point"
                << " cloud to the newest input pose guess." << std::endl;
        report(INPUTS_NOT_ALIGNED);

        return false;
    } else {
        poseGuess_ = basePoseGuess.getTransform();

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
        saveGridMap(gaSlam_.getRawMap().getGridMap(), _saveMapPath.rvalue());
        savePose(gaSlam_.getPose(), _savePosePath.rvalue());
    }

    if (_cloudDebugEnabled.rvalue()) {
        BaseCloud processedBaseCloud, mapBaseCloud;

        GaSlamBaseConverter::convertPCLToBaseCloud(processedBaseCloud,
                gaSlam_.getProcessedCloud());
        GaSlamBaseConverter::convertMapToBaseCloud(mapBaseCloud,
                gaSlam_.getRawMap());

        _processedCloud.write(processedBaseCloud);
        _mapCloud.write(mapBaseCloud);
    }
}

}  // namespace ga_slam

