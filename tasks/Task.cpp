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

void Task::poseGuessTransformerCallback(
        const BaseTime& timestamp,
        const BasePose& basePoseGuess) {
    Pose poseGuess, bodyToGroundTF;

    if (!_body2ground.get(timestamp, bodyToGroundTF, false)) {
        RTT::log(RTT::Error) << "Body to ground TF not found" << RTT::endlog();
        error(TRANSFORM_NOT_FOUND);
        return;
    }

    poseGuess = basePoseGuess.getTransform();

    gaSlam_.poseCallback(poseGuess, bodyToGroundTF);

    if (_debugInfoEnabled.rvalue()) outputDebugInfo();
}

void Task::hazcamCloudTransformerCallback(
        const BaseTime& timestamp,
        const BaseCloud& baseHazcamCloud) {
    Pose hazcamToBodyTF;

    if (!_hazcam2body.get(timestamp, hazcamToBodyTF, false)) {
        RTT::log(RTT::Error) << "HazCam to body TF not found" << RTT::endlog();
        error(TRANSFORM_NOT_FOUND);
        return;
    }

    cloudCallback(baseHazcamCloud, hazcamToBodyTF);
}

void Task::loccamCloudTransformerCallback(
        const BaseTime& timestamp,
        const BaseCloud& baseLoccamCloud) {
    Pose loccamToBodyTF;

    if (!_loccam2body.get(timestamp, loccamToBodyTF, false)) {
        RTT::log(RTT::Error) << "LocCam to body TF not found" << RTT::endlog();
        error(TRANSFORM_NOT_FOUND);
        return;
    }

    cloudCallback(baseLoccamCloud, loccamToBodyTF);
}

void Task::pancamCloudTransformerCallback(
        const BaseTime& timestamp,
        const BaseCloud& basePancamCloud) {
    BasePose basePancamToBodyTF;

    if (_pancamTransformation.read(basePancamToBodyTF) != RTT::NewData) {
        RTT::log(RTT::Error) << "PanCam to body TF not found" << RTT::endlog();
        error(TRANSFORM_NOT_FOUND);
        return;
    }

    cloudCallback(basePancamCloud, basePancamToBodyTF.getTransform());
}

void Task::cloudCallback(
        const BaseCloud& baseCloud,
        const Pose& sensorToBodyTF) {
    Cloud::Ptr cloud(new Cloud);
    GaSlamBaseConverter::convertBaseCloudToPCL(baseCloud, cloud);

    gaSlam_.cloudCallback(cloud, sensorToBodyTF);

    if (_debugInfoEnabled.rvalue()) outputDebugInfo();
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

