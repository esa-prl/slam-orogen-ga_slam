#include "GaSlamBaseConverter.hpp"

namespace ga_slam {

void GaSlamBaseConverter::convertBaseCloudToPCL(
        const BaseCloud& baseCloud,
        PointCloud::Ptr& pclCloud) {
    pclCloud->clear();
    pclCloud->reserve(baseCloud.points.size());
    pclCloud->is_dense = true;
    pclCloud->header.stamp = baseCloud.time.toMicroseconds();

    for (const auto& point : baseCloud.points)
        pclCloud->push_back(pcl::PointXYZ(point.x(), point.y(), point.z()));
}

void GaSlamBaseConverter::convertPCLToBaseCloud(
        BaseCloud& baseCloud,
        const PointCloud::ConstPtr& pclCloud) {
    baseCloud.points.clear();
    baseCloud.points.reserve(pclCloud->size());
    baseCloud.time.fromMicroseconds(pclCloud->header.stamp);

    for (const auto& point : pclCloud->points)
        baseCloud.points.push_back(base::Point(point.x, point.y, point.z));
}

void GaSlamBaseConverter::convertMapToBaseImage(
        BaseImage& image,
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

void GaSlamBaseConverter::convertBaseImageToMap(
        const BaseImage& image,
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

void GaSlamBaseConverter::convertMapToBaseCloud(
        BaseCloud& baseCloud,
        const Map& map) {
    baseCloud.points.clear();
    baseCloud.points.reserve(map.getSize().x() * map.getSize().y());
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

