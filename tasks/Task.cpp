#include "Task.hpp"

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
        const base::Time& timestamp,
        const base::samples::Pointcloud& inputBaseCloud) {
    if (!readPoseAndTF(timestamp))
        return;

    inputPose_ = inputBasePose_.getTransform();
    convertBaseCloudToPCL(inputBaseCloud, inputPCLCloud_);

    gaSlam_.registerData(inputPose_, cameraToMapTF_, inputPCLCloud_);

    if (_debugEnabled.rvalue()) {
        saveGridMap(gaSlam_.getRawMap(), _savePath.rvalue());

        convertMapToBaseFrame(rawMapBaseFrame_, gaSlam_.getRawMap(),
                _minElevation.rvalue(), _maxElevation.rvalue());
        convertPCLToBaseCloud(filteredBaseCloud_,
                gaSlam_.getFilteredPointCloud());
        convertMapToBaseCloud(mapBaseCloud_, gaSlam_.getRawMap());

        _rawElevationMap.write(rawMapBaseFrame_);
        _filteredPointCloud.write(filteredBaseCloud_);
        _mapPointCloud.write(mapBaseCloud_);
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
                << " cloud to the newest input pose." << std::endl;
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

void Task::convertBaseCloudToPCL(
        const base::samples::Pointcloud& baseCloud,
        PointCloud::Ptr& pclCloud) {
    pclCloud->clear();
    pclCloud->is_dense = true;
    pclCloud->header.stamp = baseCloud.time.toMicroseconds();

    for (const auto& point : baseCloud.points)
        pclCloud->push_back(pcl::PointXYZ(point.x(), point.y(), point.z()));
}

void Task::convertPCLToBaseCloud(
        base::samples::Pointcloud& baseCloud,
        const PointCloud::ConstPtr& pclCloud) {
    baseCloud.points.clear();
    baseCloud.time.fromMicroseconds(pclCloud->header.stamp);

    for (const auto& point : pclCloud->points)
        baseCloud.points.push_back(base::Point(point.x, point.y, point.z));
}

void Task::convertMapToBaseFrame(
        base::samples::frame::Frame& frame,
        const Map& map,
        const double& minElevation,
        const double& maxElevation) {
    frame.init(map.getSize()(0), map.getSize()(1));
    frame.time.fromMicroseconds(map.getTimestamp());

    std::vector<uint8_t> image;
    const grid_map::Matrix& data = map.get("meanZ");

    for (grid_map::GridMapIterator it(map); !it.isPastEnd(); ++it) {
        const grid_map::Index index(*it);
        image.push_back(static_cast<uint8_t>(data(index(1), index(0))));
    }

    for (auto& value : image)
        value = 255. * (value - minElevation) / (maxElevation - minElevation);

    frame.setImage(image);
}

void Task::convertBaseFrameToMap(
        const base::samples::frame::Frame& frame,
        Map& map,
        const double& resolution,
        const double& positionX,
        const double& positionY,
        const double& minElevation,
        const double& maxElevation) {
    map.add("meanZ");
    map.setBasicLayers({"meanZ"});
    map.clearBasic();
    map.setTimestamp(frame.time.toMicroseconds());
    map.setGeometry(grid_map::Length(frame.getWidth(), frame.getHeight()),
            resolution, grid_map::Position(positionX, positionY));

    grid_map::Matrix& data = map.get("meanZ");

    for (grid_map::GridMapIterator it(map); !it.isPastEnd(); ++it) {
        const int i = it.getLinearIndex();

        data(i) = (frame.image[i] / 255.) * (maxElevation - minElevation) +
                minElevation;
    }
}

void Task::convertMapToBaseCloud(
        base::samples::Pointcloud& baseCloud,
        const Map& map) {
    baseCloud.points.clear();
    baseCloud.time.fromMicroseconds(map.getTimestamp());

    const grid_map::Matrix& data = map.get("meanZ");
    grid_map::Position point;

    for (grid_map::GridMapIterator it(map); !it.isPastEnd(); ++it) {
        const grid_map::Index index(*it);

        map.getPosition(index, point);
        baseCloud.points.push_back(base::Point(
                point.x(), point.y(), data(index(0), index(1))));
    }
}

}  // namespace ga_slam

