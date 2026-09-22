#ifndef DRIVING_ROBOT__SCAN_SECTORS_HPP_
#define DRIVING_ROBOT__SCAN_SECTORS_HPP_

#include "sensor_msgs/msg/laser_scan.hpp"
#include <cmath>
#include <limits>

struct Sector
{
    int first;
    int last;
    const char *name;
};

// the lidar publishes 720 beams over 360 degrees, starting from the rear
constexpr Sector REAR_RIGHT = {0, 180, "rear right"};
constexpr Sector FRONT_RIGHT = {180, 360, "front right"};
constexpr Sector FRONT_LEFT = {360, 540, "front left"};
constexpr Sector REAR_LEFT = {540, 719, "rear left"};

constexpr Sector SECTORS[4] = {REAR_RIGHT, FRONT_RIGHT, FRONT_LEFT, REAR_LEFT};

inline double min_scan_in_sector(const sensor_msgs::msg::LaserScan::SharedPtr &scan, Sector sector)
{
    double minimum = std::numeric_limits<double>::infinity();

    // find the minimum over the beams of the sector
    for (int i = sector.first; i <= sector.last; i++)
    {
        double current = scan->ranges[i];
        if (std::isfinite(current) && current < minimum)
        {
            minimum = current;
        }
    }
    return minimum;
}

#endif
