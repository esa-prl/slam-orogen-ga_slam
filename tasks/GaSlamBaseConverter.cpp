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
    baseCloud.time.fromMicroseconds(pclCloud->header.stamp);

    for (const auto& point : pclCloud->points)
        baseCloud.points.push_back(base::Point(point.x, point.y, point.z));
}

void GaSlamBaseConverter::convertMapToBaseImage(
        BaseImage& image,
        const Map& map) {
    const auto params = map.getParameters();

    image.width = params.sizeX;
    image.height = params.sizeY;
    image.data.clear();
    image.data.reserve(image.width * image.height);
    image.time.fromMicroseconds(map.getTimestamp());

    const auto& meanData = map.getMeanZ();

    for (auto&& it = map.begin(); !it.isPastEnd(); ++it)
        image.data.push_back(meanData(it.getLinearIndex()));
}

void GaSlamBaseConverter::convertMapToBaseCloud(
        BaseCloud& baseCloud,
        const Map& map) {
    const auto params = map.getParameters();

    baseCloud.points.clear();
    baseCloud.points.reserve(params.sizeX * params.sizeY);
    baseCloud.time.fromMicroseconds(map.getTimestamp());

    const auto& meanData = map.getMeanZ();
    Eigen::Vector3d point;

    for (auto&& it = map.begin(); !it.isPastEnd(); ++it) {
        map.getPointFromArrayIndex(*it, meanData, point);
        baseCloud.points.push_back(point);
    }
}

}  // namespace ga_slam

