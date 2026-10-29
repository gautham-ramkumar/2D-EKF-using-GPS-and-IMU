#include "utm.hpp"
#include <cmath>
#include <iostream>

Utm latLonToUtm(double lat, double lon) {
    Utm utm;
    const double a = 6378137.0; // WGS84 semi-major axis
    const double f = 1.0 / 298.257223563; // WGS84 flattening
    const double e2 = 2 * f - f * f; // eccentricity squared
    const double k0 = 0.9996; // scale factor
    const double x0 = 500000.0; // false easting
    const double y0 = 0.0; // false northing (northern hemisphere)
    const double ep2 = e2 / (1.0 - e2); // second eccentricity squared

    // Calculate the UTM zone based on the longitude
    utm.zone = static_cast<int>((lon + 180.0) / 6.0) + 1;

    const double lon0 = (utm.zone - 1) * 6.0 - 180.0 + 3.0; // central meridian of the UTM zone

    // Convert degrees to radians
    const double phi = lat * M_PI / 180.0;
    const double lambda = lon * M_PI / 180.0;

    //Helper values for the conversion
    const double sinphi = std::sin(phi);
    const double cosphi = std::cos(phi);
    const double tanphi = std::tan(phi);
    const double N = a / std::sqrt(1.0 - e2 * sinphi * sinphi);
    const double T = tanphi * tanphi;
    const double C = e2 * cosphi * cosphi / (1.0 - e2);
    const double A = (lambda - lon0 * M_PI / 180.0)*cosphi;

    // Distance from the central meridian of the UTM zone
    const double M = a * ((1.0 - e2 / 4.0 - 3.0 * e2 * e2 / 64.0 - 5.0 * e2 * e2 * e2 / 256.0) * phi
                          - (3.0 * e2 / 8.0 + 3.0 * e2 * e2 / 32.0 + 45.0 * e2 * e2 * e2 / 1024.0) * std::sin(2.0 * phi)
                          + (15.0 * e2 * e2 / 256.0 + 45.0 * e2 * e2 * e2 / 1024.0) * std::sin(4.0 * phi)
                          - (35.0 * e2 * e2 * e2 / 3072.0) * std::sin(6.0 * phi));

    // Calculate the UTM coordinates based on the helper values
    utm.easting = k0 * N * (A + (1.0 - T + C) * A * A * A / 6.0 + (5.0 - 18.0 * T + T * T + 72.0 * C - 58.0 * ep2) * A * A * A * A * A / 120.0) + x0;
    utm.northing = k0 * (M + N * tanphi * (A * A / 2.0 + (5.0 - T + 9.0 * C + 4.0 * C * C) * A * A * A * A / 24.0 + (61.0 - 58.0 * T + T * T + 600.0 * C - 330.0 * ep2) * A * A * A * A * A * A / 720.0)) + y0;

    if (lat < 0.0) {
        utm.northing += 10000000.0; // add 10 million meters for southern hemisphere
    }

    return utm;
}

std::vector<Gpspt> convertGpsToMeters(const std::vector<Gpspt>& fixes) {
    std::vector<Gpspt> meters;
    for (const auto& fix : fixes) {
        Utm utm = latLonToUtm(fix.north, fix.east);
        Gpspt gpspt;
        gpspt.t = fix.t;
        gpspt.east = utm.easting;
        gpspt.north = utm.northing;
        meters.push_back(gpspt);
    }
    return meters;
}

std::vector<Gpspt> removeTeleports(const std::vector<Gpspt>& points) {
    std::vector<Gpspt> filtered;
    for (size_t i = 0; i < points.size(); ++i) {
        if (i == 0) {
            filtered.push_back(points[i]);
        } else {
            double dx = points[i].east - points[i - 1].east;
            double dy = points[i].north - points[i - 1].north;
            double distance = std::sqrt(dx * dx + dy * dy);
            double dt = std::max(points[i].t - points[i - 1].t, 1e-3);
            double speed = distance / dt;
            if (speed < 40.0) { // threshold for teleport detection
                filtered.push_back(points[i]);
            }
        }
    }
    return filtered;
}

std::vector<Gpspt> makeRelative(const std::vector<Gpspt>& points) {
    std::vector<Gpspt> relative;
    if (points.empty()) {
        return relative;
    }
    double originEast = points[0].east;
    double originNorth = points[0].north;
    for (const auto& point : points) {
        Gpspt rel;
        rel.t = point.t;
        rel.east = point.east - originEast;
        rel.north = point.north - originNorth;
        relative.push_back(rel);
    }
    return relative;
}