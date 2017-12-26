#include "Task.hpp"
#include "GaSlamBaseConverter.hpp"

#include "grid_map_cereal/GridMapCereal.hpp"

namespace ga_slam {

bool Task::configureHook(void) {
    if (!TaskBase::configureHook()) return false;

    gaSlam_.setParameters(_mapLengthX.rvalue(), _mapLengthY.rvalue(),
            _mapResolution.rvalue(), _minElevation.rvalue(),
            _maxElevation.rvalue(), _voxelSize.rvalue(), _numParticles.rvalue(),
            _initialSigmaX.rvalue(), _initialSigmaY.rvalue(),
            _initialSigmaYaw.rvalue(), _predictSigmaX.rvalue(),
            _predictSigmaY.rvalue(), _predictSigmaYaw.rvalue());

    return true;
}

void Task::cloudTransformerCallback(
        const BaseTime& timestamp,
        const BaseCloud& baseCloud) {
    Pose poseGuess, sensorToBodyTF, bodyToGroundTF;

    if (!readPortsAndTF(timestamp, poseGuess, sensorToBodyTF, bodyToGroundTF))
        return;

    Cloud::Ptr cloud(new Cloud);
    GaSlamBaseConverter::convertBaseCloudToPCL(baseCloud, cloud);

    gaSlam_.cloudCallback(cloud, sensorToBodyTF, bodyToGroundTF, poseGuess);

    if (_debugInfoEnabled.rvalue()) outputDebugInfo();
}

bool Task::readPortsAndTF(
        const BaseTime& timestamp,
        Pose& poseGuess,
        Pose& sensorToBodyTF,
        Pose& bodyToGroundTF) {
    if (!_slamSensor2body.get(timestamp, sensorToBodyTF, false) ||
            !_body2ground.get(timestamp, bodyToGroundTF, false)) {
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
        poseGuess = basePoseGuess.getTransform();

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

