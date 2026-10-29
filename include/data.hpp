#pragma once

#include <ostream>
#include <string>
#include <vector>

struct ImuSample {
    double t = 0.0;
    double ax = 0.0;
    double ay = 0.0;
    double az = 0.0;
    double gx = 0.0;
    double gy = 0.0;
    double gz = 0.0;

};

struct GpsFix {
    double t = 0.0;
    double lat = 0.0;
    double lon = 0.0;
    double alt = 0.0;
};

struct ImuInput {
    double t = 0.0;
    double ax = 0.0;
    double omega = 0.0;
};

std::ostream& operator<<(std::ostream& os, const ImuSample& s);
std::ostream& operator<<(std::ostream& os, const GpsFix& g);
std::ostream& operator<<(std::ostream& os, const ImuInput& i);

std::vector<ImuSample> loadImu(const std::string& path);
std::vector<GpsFix> loadGps(const std::string& path);

using GpsList = std::vector<GpsFix>;
using ImuList = std::vector<ImuSample>;
using ImuInputList = std::vector<ImuInput>;

GpsList removeNaNFixes(const GpsList& fixes);

ImuInputList prepareImu(const ImuList& raw);
// Wheel speed from /vehicle/twist (Dataspeed DBW), 50 Hz.
struct WheelSpeed {
    double t = 0.0;
    double v = 0.0;    // forward speed (m/s)
};

std::vector<WheelSpeed> loadWheel(const std::string& path);
