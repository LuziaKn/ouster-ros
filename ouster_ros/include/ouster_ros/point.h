/**
 * @file
 * @brief PCL point datatype for use with ouster sensors
 */

#pragma once
#define PCL_NO_PRECOMPILE
#include <pcl/point_types.h>

#include <Eigen/Eigen>
#include <chrono>
#include <functional>

#include "ouster/lidar_scan.h"

namespace ouster_ros {

struct EIGEN_ALIGN16 Point_Small {
    PCL_ADD_POINT4D;
    uint16_t reflectivity;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
struct EIGEN_ALIGN16 Point {
    PCL_ADD_POINT4D;
    float intensity;
    uint32_t t;
    uint16_t reflectivity;
    uint8_t ring;
    uint16_t ambient;
    uint32_t range;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
}  // namespace ouster_ros

// clang-format off
POINT_CLOUD_REGISTER_POINT_STRUCT(ouster_ros::Point,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    // use std::uint32_t to avoid conflicting with pcl::uint32_t
    (std::uint32_t, t, t)
    (std::uint16_t, reflectivity, reflectivity)
    (std::uint8_t, ring, ring)
    (std::uint16_t, ambient, ambient)
    (std::uint32_t, range, range)
)
POINT_CLOUD_REGISTER_POINT_STRUCT(ouster_ros::Point_Small,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (std::uint16_t, reflectivity, reflectivity)
)

// clang-format on
