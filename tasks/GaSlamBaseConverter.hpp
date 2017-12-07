#ifndef _GASLAM_TYPE_CONVERTER_HPP_
#define _GASLAM_TYPE_CONVERTER_HPP_

#include "ga_slam/Task.hpp"

namespace ga_slam {

class GaSlamBaseConverter {
  public:
    static void convertBaseCloudToPCL(
            const BaseCloud& baseCloud,
            PointCloud::Ptr& pclCloud);

    static void convertPCLToBaseCloud(
            BaseCloud& baseCloud,
            const PointCloud::ConstPtr& pclCloud);

    static void convertBaseImageToMap(
            const BaseImage& image,
            Map& map,
            const double& resolution,
            const double& positionX,
            const double& positionY);

    static void convertMapToBaseImage(
            BaseImage& image,
            const Map& map);

    static void convertMapToBaseCloud(
            BaseCloud& baseCloud,
            const Map& map);
};

}  // namespace ga_slam

#endif  // _GASLAM_TYPE_CONVERTER_HPP_

