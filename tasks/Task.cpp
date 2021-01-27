#include "Task.hpp"
#include "GaSlamBaseConverter.hpp"

#include "ga_slam_cereal/GridMapCereal.h"

#include <mutex>

namespace ga_slam {

bool Task::configureHook(void) {
    if (!TaskBase::configureHook()) return false;

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

    gaSlam_.configure(_mapLength.rvalue(), _mapResolution.rvalue(),
            _minElevation.rvalue(), _maxElevation.rvalue(), _voxelSize.rvalue(),
            _depthSigmaCoeff1.rvalue(), _depthSigmaCoeff2.rvalue(),
            _depthSigmaCoeff3.rvalue(),
            _numParticles.rvalue(), _resampleFrequency.rvalue(),
            _initialSigmaX.rvalue(), _initialSigmaY.rvalue(),
            _initialSigmaYaw.rvalue(), _predictSigmaX.rvalue(),
            _predictSigmaY.rvalue(), _predictSigmaYaw.rvalue(),
            _traversedDistanceThreshold.rvalue(), _minSlopeThreshold.rvalue(),
            _slopeSumThresholdMultiplier.rvalue(),
            _matchAcceptanceThreshold.rvalue(), _matchYaw.rvalue(),
            _matchYawStep.rvalue(), _matchYawRange.rvalue(),
            _orbiterMapLength.rvalue(), _orbiterMapResolution.rvalue());

    return true;
}

void Task::updateHook(void) {
    TaskBase::updateHook();

    if (isFutureReady(odometryPoseFuture_) &&
            _odometryPose.read(baseOdometryPose_) == RTT::NewData) {
        Pose odometryPose = baseOdometryPose_.getTransform();
        Pose odometryDeltaPose = lastOdometryPose_.inverse() * odometryPose;
        lastOdometryPose_ = odometryPose;

        odometryPoseFuture_ = std::async(std::launch::async,
                &GaSlam::poseCallback, &gaSlam_, odometryDeltaPose);
    }

    if (isFutureReady(imuOrientationFuture_) &&
            _imuOrientation.read(baseImuOrientation_) == RTT::NewData)
        imuOrientationFuture_ = std::async(std::launch::async,
                &GaSlam::imuCallback, &gaSlam_,
                baseImuOrientation_.getTransform());

    if (isFutureReady(hazcamCloudFuture_) &&
            _hazcamCloud.read(hazcamCloud_) == RTT::NewData)
        hazcamCloudFuture_ = std::async(std::launch::async,
                &Task::cloudCallback, this, hazcamCloud_, hazcamToBodyTF_);

    if (isFutureReady(loccamCloudFuture_) &&
            _loccamCloud.read(loccamCloud_) == RTT::NewData)
        loccamCloudFuture_ = std::async(std::launch::async,
                &Task::cloudCallback, this, loccamCloud_, loccamToBodyTF_);

    if (isFutureReady(pancamCloudFuture_) &&
            _pancamCloud.read(pancamCloud_) == RTT::NewData &&
            _pancamTransformation.read(basePancamToBodyTF_) == RTT::NewData)
        pancamCloudFuture_ = std::async(std::launch::async,
                &Task::cloudCallback, this, pancamCloud_,
                basePancamToBodyTF_.getTransform());

    if (isFutureReady(orbiterCloudFuture_) &&
            _orbiterCloudPose.read(orbiterCloudPose_) == RTT::NewData &&
            _orbiterCloud.read(orbiterCloud_) == RTT::NewData)
        orbiterCloudFuture_ = std::async(std::launch::async, [&] {
            Cloud::Ptr cloud(new Cloud);
            GaSlamBaseConverter::convertBaseCloudToPCL(orbiterCloud_, cloud);
            gaSlam_.createGlobalMap(cloud, orbiterCloudPose_.getTransform());
        });

    outputPortData();

    if (_debugInfoEnabled.rvalue()) outputDebugData();
}

void Task::cloudCallback(
        const BaseCloud& baseCloud,
        const Pose& sensorToBodyTF) {
    Cloud::Ptr cloud(new Cloud);
    GaSlamBaseConverter::convertBaseCloudToPCL(baseCloud, cloud);

    gaSlam_.cloudCallback(cloud, sensorToBodyTF);
}

void Task::outputPortData(void) {
        BaseImage localElevationMapMeanBaseImage, localElevationMapVarianceBaseImage, globalElevationMapMeanBaseImage, globalElevationMapVarianceBaseImage;

        std::unique_lock<std::mutex> localGuard(gaSlam_.getLocalMapMutex());
        GaSlamBaseConverter::convertMapToBaseImage(localElevationMapMeanBaseImage, localElevationMapVarianceBaseImage, gaSlam_.getLocalMap());
        localGuard.unlock();

        std::unique_lock<std::mutex> globalGuard(gaSlam_.getGlobalMapMutex());
        GaSlamBaseConverter::convertMapToBaseImage(globalElevationMapMeanBaseImage, globalElevationMapVarianceBaseImage, gaSlam_.getGlobalMap());
        globalGuard.unlock();

        BasePose estimatedPoseBase;
        estimatedPoseBase.setTransform(gaSlam_.getPose());

        _localElevationMapMean.write(localElevationMapMeanBaseImage);
        _localElevationMapVariance.write(localElevationMapVarianceBaseImage);
        _globalElevationMapMean.write(globalElevationMapMeanBaseImage);
        _globalElevationMapVariance.write(globalElevationMapVarianceBaseImage);
        _estimatedPose.write(estimatedPoseBase);

        double estimationError = sqrt(pow(estimatedPoseBase.position.x()-orbiterCloudPose_.position.x(),2) + pow(estimatedPoseBase.position.y()-orbiterCloudPose_.position.y(),2));
        _estimationError.write(estimationError);
}

void Task::outputDebugData(void) {
    if (_cloudDebugEnabled.rvalue()) {
        BaseCloud mapBaseCloud;

        std::unique_lock<std::mutex> guard(gaSlam_.getLocalMapMutex());
        GaSlamBaseConverter::convertMapToBaseCloud(mapBaseCloud,
                gaSlam_.getLocalMap());
        guard.unlock();

        _mapCloud.write(mapBaseCloud);
    }

    if (_serializationDebugEnabled.rvalue()) {
        savePose(lastOdometryPose_, _odometryPosePath.rvalue());

        savePose(gaSlam_.getPose(), _slamPosePath.rvalue());

        savePose(orbiterCloudPose_.getTransform(), _globalPosePath.rvalue());

        saveArray(gaSlam_.getParticlesArray(), _particlesArrayPath.rvalue());

        std::unique_lock<std::mutex> localMapGuard(gaSlam_.getLocalMapMutex());
        saveGridMap(gaSlam_.getLocalMap().getGridMap(), _localMapPath.rvalue());
        localMapGuard.unlock();

        std::lock_guard<std::mutex> globalMapGuard(gaSlam_.getGlobalMapMutex());
        saveGridMap(gaSlam_.getGlobalMap().getGridMap(),
                _globalMapPath.rvalue());
    }
}

}  // namespace ga_slam

