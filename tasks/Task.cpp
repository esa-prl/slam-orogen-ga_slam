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

    if (_debugInfoEnabled.rvalue()) {
        if (_rawMapDebugEnabled.rvalue()) {
            convertMapToBaseDistanceImage(rawMapBaseDistanceImage_,
                    gaSlam_.getRawMap());

            _rawElevationMap.write(rawMapBaseDistanceImage_);
        }

        if (_serializationDebugEnabled.rvalue())
            saveGridMap(gaSlam_.getRawMap(), _savePath.rvalue());

        if (_pointCloudDebugEnabled.rvalue()) {
            convertPCLToBaseCloud(filteredBaseCloud_,
                    gaSlam_.getFilteredPointCloud());
            convertMapToBaseCloud(mapBaseCloud_, gaSlam_.getRawMap());

            _filteredPointCloud.write(filteredBaseCloud_);
            _mapPointCloud.write(mapBaseCloud_);
        }
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

void Task::convertMapToBaseDistanceImage(
        base::samples::DistanceImage& image,
        const Map& map) {
    image.width = map.getSize()(0);
    image.height = map.getSize()(1);
    image.data.clear();
    image.data.reserve(image.width * image.height);
    image.time.fromMicroseconds(map.getTimestamp());

    const grid_map::Matrix& mapData = map.get("meanZ");

    for (grid_map::GridMapIterator it(map); !it.isPastEnd(); ++it) {
        const auto& index = it.getLinearIndex();
        image.data.push_back(mapData(index));
    }
}

void Task::convertBaseDistanceImageToMap(
        const base::samples::DistanceImage& image,
        Map& map,
        const double& resolution,
        const double& positionX,
        const double& positionY) {
    map.add("meanZ");
    map.setBasicLayers({"meanZ"});
    map.clearBasic();
    map.setTimestamp(image.time.toMicroseconds());
    map.setGeometry(grid_map::Length(image.width, image.height),
            resolution, grid_map::Position(positionX, positionY));

    grid_map::Matrix& mapData = map.get("meanZ");

    for (grid_map::GridMapIterator it(map); !it.isPastEnd(); ++it) {
        const auto& index = it.getLinearIndex();
        mapData(index) = image.data[index];
    }
}

void Task::convertMapToBaseCloud(
        base::samples::Pointcloud& baseCloud,
        const Map& map) {
    baseCloud.points.clear();
    baseCloud.time.fromMicroseconds(map.getTimestamp());

    const grid_map::Matrix& mapData = map.get("meanZ");
    grid_map::Position point;

    for (grid_map::GridMapIterator it(map); !it.isPastEnd(); ++it) {
        const grid_map::Index index(*it);

        map.getPosition(index, point);
        baseCloud.points.push_back(base::Point(
                point.x(), point.y(), mapData(index(0), index(1))));
    }
}

}  // namespace ga_slam

