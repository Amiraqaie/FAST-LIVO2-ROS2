#pragma once

#include <pcl/register_point_struct.h>
#include <cstdint>

struct MapPoint
{
  float x, y, z, intensity;
  float normal_x, normal_y, normal_z, curvature;
  std::uint32_t rgb;
  std::uint8_t has_color;
};

POINT_CLOUD_REGISTER_POINT_STRUCT(MapPoint,
    (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)
    (float, normal_x, normal_x)(float, normal_y, normal_y)(float, normal_z, normal_z)
    (float, curvature, curvature)(std::uint32_t, rgb, rgb)(std::uint8_t, has_color, has_color))
