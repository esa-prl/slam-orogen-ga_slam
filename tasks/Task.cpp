#include "Task.hpp"
#include "GaSlamBaseConverter.hpp"

#include "grid_map_cereal/GridMapCereal.hpp"

namespace ga_slam {

bool Task::configureHook(void) {
    if (!TaskBase::configureHook()) return false;

    if (!_body2ground.get(BaseTime::now(), bodyToGroundTF_, false)) {
        RTT::log(RTT::Error) << "Body to ground TF not found" << RTT::endlog();
        error(TRANSFORM_NOT_FOUND);
        return false;
    }

    if (!_hazcam2body.get(BaseTime::now(), hazcamToBodyTF_, false)) {
        RTT::log(RTT::Error) << "HazCam to body TF not found" << RTT::endlog();
        error(TRANSFORM_NOT_FOUND);
        return false;
    }

    if (!_loccam2body.get(BaseTime::now(), loccamToBodyTF_, false)) {
        RTT::log(RTT::Error) << "LocCam to body TF not found" << RTT::endlog();
        error(TRANSFORM_NOT_FOUND);
        return false;
    }

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
    if (!isFutureReady(poseGuessFuture_)) return;

    poseGuessFuture_ = std::async(std::launch::async, &GaSlam::poseCallback,
            &gaSlam_, basePoseGuess.getTransform(), bodyToGroundTF_);
}

void Task::hazcamCloudTransformerCallback(
        const BaseTime& timestamp,
        const BaseCloud& baseHazcamCloud) {
    std::cout << "[GA SLAM] HazCam Cloud received!" << std::endl;

    if (!isFutureReady(hazcamCloudFuture_)) return;

    hazcamCloudFuture_ = std::async(std::launch::async,
            &Task::cloudCallback, this, baseHazcamCloud, hazcamToBodyTF_);
}

void Task::loccamCloudTransformerCallback(
        const BaseTime& timestamp,
        const BaseCloud& baseLoccamCloud) {
    std::cout << "[GA SLAM] LocCam Cloud received!" << std::endl;

    if (!isFutureReady(loccamCloudFuture_)) return;

    loccamCloudFuture_ = std::async(std::launch::async,
            &Task::cloudCallback, this, baseLoccamCloud, loccamToBodyTF_);
}

void Task::pancamCloudTransformerCallback(
        const BaseTime& timestamp,
        const BaseCloud& basePancamCloud) {
    std::cout << "[GA SLAM] PanCam Cloud received!" << std::endl;

    if (_pancamTransformation.read(basePancamToBodyTF_) != RTT::NewData) return;

    if (!isFutureReady(pancamCloudFuture_)) return;

    pancamCloudFuture_ = std::async(std::launch::async, &Task::cloudCallback,
            this, basePancamCloud,
            basePancamToBodyTF_.getTransform());
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

