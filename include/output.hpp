#pragma once

#include <string>
#include <vector>

#include "ekf2d.hpp"
#include "utm.hpp"

// One logged filter state, for plotting.
struct TrajRow {
    double t = 0.0;
    double px = 0.0;
    double py = 0.0;
    double theta = 0.0;
    double v = 0.0;
    double stdPx = 0.0;   // 1-sigma position uncertainty, sqrt(P(0,0))
    double stdPy = 0.0;   // sqrt(P(1,1))
};

// Snapshot the filter's current state at time t.
TrajRow makeTrajRow(double t, const Ekf2D& ekf);

// Write rows as CSV with a header. Creates the folder if needed; throws on failure.
void writeTrajectory(const std::string& path, const std::vector<TrajRow>& rows);
// What happened to each GPS point, written as a column for the plot.
enum GpsStatus { GpsUsed = 0, GpsGated = 1, GpsHeldOut = 2, GpsOutage = 3 };
void writeGpsPoints(const std::string& path, const std::vector<Gpspt>& points,
                    const std::vector<int>& status);
