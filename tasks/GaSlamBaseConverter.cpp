#include "GaSlamBaseConverter.hpp"

namespace ga_slam {

void GaSlamBaseConverter::convertBaseCloudToPCL(
        const BaseCloud& baseCloud,
        Cloud::Ptr& pclCloud) {
    pclCloud->clear();
    pclCloud->reserve(baseCloud.points.size());
    pclCloud->is_dense = true;
    pclCloud->header.stamp = baseCloud.time.toMicroseconds();

    for (const auto& point : baseCloud.points)
        pclCloud->push_back(pcl::PointXYZ(point.x(), point.y(), point.z()));
}

void GaSlamBaseConverter::convertPCLToBaseCloud(
        BaseCloud& baseCloud,
        const Cloud::ConstPtr& pclCloud) {
    baseCloud.points.clear();
    baseCloud.points.reserve(pclCloud->size());
    baseCloud.time = BaseTime::fromMicroseconds(pclCloud->header.stamp);

    for (const auto& point : pclCloud->points)
        baseCloud.points.push_back(base::Point(point.x, point.y, point.z));
}

void GaSlamBaseConverter::convertMapToBaseImage(
        BaseImage& mean_image,
        BaseImage& var_image,
        const Map& map) {
    const auto params = map.getParameters();
    const auto meanData = map.getMeanZ();
    const auto varData = map.getVarianceZ();

    /* init mean image */
    mean_image.width = params.size;
    mean_image.height = params.size;
    mean_image.data.clear();
    mean_image.data.resize(mean_image.width * mean_image.height, NAN);
    mean_image.time = BaseTime::fromMicroseconds(map.getTimestamp());

    /* init var image */
    var_image.width = params.size;
    var_image.height = params.size;
    var_image.data.clear();
    var_image.data.resize(var_image.width * var_image.height, NAN);
    var_image.time = BaseTime::fromMicroseconds(map.getTimestamp());

    for (auto&& it = map.begin(); !it.isPastEnd(); ++it) {
        const grid_map::Index index(*it);
        const grid_map::Index imageIndex(it.getUnwrappedIndex());

        const float mean_value = meanData(index(0), index(1));
        mean_image.data[imageIndex(0) * mean_image.height + imageIndex(1)] = mean_value;

        const float var_value = varData(index(0), index(1));
        var_image.data[imageIndex(0) * var_image.height + imageIndex(1)] = var_value;
    }
}

void GaSlamBaseConverter::convertMapToBaseCloud(
        BaseCloud& baseCloud,
        const Map& map) {
    const auto params = map.getParameters();

    baseCloud.points.clear();
    baseCloud.points.reserve(params.size * params.size);
    baseCloud.time = BaseTime::fromMicroseconds(map.getTimestamp());

    const auto& meanData = map.getMeanZ();
    Eigen::Vector3d point;

    for (auto&& it = map.begin(); !it.isPastEnd(); ++it) {
        map.getPointFromArrayIndex(*it, meanData, point);
        baseCloud.points.push_back(point);
    }
}

}  // namespace ga_slam

