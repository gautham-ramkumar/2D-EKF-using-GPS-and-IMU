#pragma once

#include <vector>

#include "utm.hpp"

// Which GPS points the filter never sees, so they can be used to score it.
// Same scheme as EKF_2D.ipynb:
//   outage:   30 s windows every 200 s (starting 100 s in) that are almost
//             fully covered by GPS. Simulates a dropout; the last point of
//             each window measures how far the IMU-only prediction drifted.
//   held-out: every 10th point (starting at index 5), outside outages. Scored
//             against the filter's prediction, never used to update it.
std::vector<bool> outageMask(const std::vector<Gpspt>& points);
std::vector<bool> heldOutMask(const std::vector<Gpspt>& points, const std::vector<bool>& outage);

// Everything the main loop records for scoring. Errors are distances (m)
// between the filter's position just before a GPS point and that point.
struct RunLog {
    std::vector<double> fixT;        // points given to the filter
    std::vector<double> priorErr;
    std::vector<bool> accepted;      // false = gated out
    std::vector<double> nis;
    std::vector<double> heldT;       // held-out points
    std::vector<double> heldErr;
    std::vector<double> outageErr;   // one per outage window, at its end
};

// Print the same score table as the notebook's "3. + retuned (final)" row.
void printScores(const RunLog& log, int numRejected, int numRecoveries);
