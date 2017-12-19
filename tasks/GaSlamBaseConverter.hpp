#pragma once

#include "ga_slam/Task.hpp"

namespace ga_slam {

class GaSlamBaseConverter {
  public:
    GaSlamBaseConverter(void) = delete;

    static void convertBaseCloudToPCL(
            const BaseCloud& baseCloud,
            Cloud::Ptr& pclCloud);

    static void convertPCLToBaseCloud(
            BaseCloud& baseCloud,
            const Cloud::ConstPtr& pclCloud);

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

