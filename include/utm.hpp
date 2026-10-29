#pragma once

#include <vector>
#include "data.hpp"

struct Utm {
    double easting = 0.0;
    double northing = 0.0;
    int zone = 0;
};

Utm latLonToUtm(double lat, double lon);

// Convert all GPS fixes to meters
struct Gpspt {
    double t = 0.0;
    double east = 0.0;
    double north = 0.0;
};

std::vector<Gpspt> convertGpsToMeters(const std::vector<Gpspt>& fixes);
std::vector<Gpspt> removeTeleports(const std::vector<Gpspt>& points);
std::vector<Gpspt> makeRelative(const std::vector<Gpspt>& points);