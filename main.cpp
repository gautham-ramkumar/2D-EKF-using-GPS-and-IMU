#include "data.hpp"
#include "utm.hpp"
#include "ekf2d.hpp"
#include "output.hpp"
#include "evaluation.hpp"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <string>

int main(int argc, char* argv[]) {
    // Run with --no-wheel to see the filter without wheel speed (IMU + GPS only).
    const bool useWheel = !(argc > 1 && std::string(argv[1]) == "--no-wheel");

    try {
        std::vector<ImuSample> imuSamples = loadImu("data/imu.csv");
        std::vector<GpsFix> gpsFixes = loadGps("data/gps.csv");
        std::vector<WheelSpeed> wheel = loadWheel("data/wheel.csv");
        std::cout << "Loaded " << imuSamples.size() << " IMU samples, " << gpsFixes.size() << " GPS fixes, "
                  << wheel.size() << " wheel speeds" << std::endl;
        std::cout << (useWheel ? "Fusing wheel speed" : "NOT fusing wheel speed (--no-wheel)") << std::endl;

        // ---- GPS: drop NaN fixes, convert to meters, drop teleports, shift to the first fix ----
        gpsFixes = removeNaNFixes(gpsFixes);

        std::vector<Gpspt> gpsptFixes;
        for (const auto& fix : gpsFixes) {
            Gpspt gpspt;
            gpspt.t = fix.t;
            gpspt.east = fix.lon;
            gpspt.north = fix.lat;
            gpsptFixes.push_back(gpspt);
        }
        std::vector<Gpspt> gpsptMeters = convertGpsToMeters(gpsptFixes);
        std::vector<Gpspt> gpsptFiltered = removeTeleports(gpsptMeters);
        std::vector<Gpspt> gpsptRelative = makeRelative(gpsptFiltered);
        std::cout << "Usable GPS points: " << gpsptRelative.size() << std::endl;

        // ---- IMU: flip gyro, remove parked biases ----
        ImuInputList imuInputs = prepareImu(imuSamples);
        std::cout << "IMU inputs: " << imuInputs.size() << std::endl;

        // ---- Initial state from GPS ----
        // Heading: direction of the first point more than 15 m from the start
        // (closer points are dominated by GPS noise). Speed: over the first 6 points.
        size_t iHeading = 0;
        double theta0 = 0.0;
        double v0 = 0.0;

        for (size_t i = 0; i < gpsptRelative.size(); i++) {
            double distance = std::sqrt(gpsptRelative[i].east * gpsptRelative[i].east
                                      + gpsptRelative[i].north * gpsptRelative[i].north);
            if (distance > 15.0) {
                iHeading = i;
                break;
            }
        }
        theta0 = std::atan2(gpsptRelative[iHeading].north, gpsptRelative[iHeading].east);
        v0 = std::sqrt(gpsptRelative[5].east * gpsptRelative[5].east
                      + gpsptRelative[5].north * gpsptRelative[5].north)
             / (gpsptRelative[5].t - gpsptRelative[0].t);
        std::cout << "Initial heading: " << iHeading << ", theta0: " << theta0 << ", v0: " << v0 << std::endl;

        // ---- Run the filter ----
        Ekf2D ekf;
        ekf.setInitialState(0.0, 0.0, theta0, v0);

        // Start at the first IMU sample at or after the first GPS fix
        // (before that, we don't know where the car is).
        size_t k0 = 1;
        while (k0 < imuInputs.size() && imuInputs[k0].t < gpsptRelative[0].t) {
            k0++;
        }

        size_t j = 0;                    // index of the next GPS point to use
        std::vector<TrajRow> trajectory; // logged every 10th IMU step, for plotting

        // ---- Evaluation: hide some GPS points from the filter to score it ----
        const std::vector<bool> outage = outageMask(gpsptRelative);
        const std::vector<bool> heldOut = heldOutMask(gpsptRelative, outage);
        RunLog log;
        std::vector<int> gpsStatus(gpsptRelative.size(), GpsUsed);   // for plotting

        size_t w = 0;                    // index of the next wheel speed to use

        for (size_t k = k0; k < imuInputs.size(); k++) {
            double dt = imuInputs[k].t - imuInputs[k - 1].t;
            if (dt <= 0.0) {
                continue;                // skip duplicate/out-of-order timestamps
            }
            dt = std::min(dt, 0.05);     // cap: never integrate more than 50 ms at once

            ekf.predict(imuInputs[k].ax, imuInputs[k].omega, dt);

            // Apply every wheel speed whose time has been reached (50 Hz, so 0 or 1 per IMU step).
            while (w < wheel.size() && wheel[w].t <= imuInputs[k].t) {
                if (useWheel) {
                    ekf.updateSpeed(wheel[w].v);
                }
                w++;
            }

            // Apply every GPS point whose time has been reached.
            while (j < gpsptRelative.size() && gpsptRelative[j].t <= imuInputs[k].t) {
                const Gpspt& p = gpsptRelative[j];
                // How far the filter's prediction is from this GPS point, before using it.
                const double err = std::hypot(ekf.state()(0) - p.east, ekf.state()(1) - p.north);

                if (outage[j]) {
                    gpsStatus[j] = GpsOutage;
                    // Hidden. Score only the last point of each outage window.
                    if (j + 1 == gpsptRelative.size() || !outage[j + 1]) {
                        log.outageErr.push_back(err);
                    }
                } else if (heldOut[j]) {
                    // Hidden. Used only for scoring.
                    gpsStatus[j] = GpsHeldOut;
                    log.heldT.push_back(p.t);
                    log.heldErr.push_back(err);
                } else {
                    log.fixT.push_back(p.t);
                    log.priorErr.push_back(err);
                    log.accepted.push_back(ekf.update(p.east, p.north));
                    if (!log.accepted.back()) {
                        gpsStatus[j] = GpsGated;
                    }
                    log.nis.push_back(ekf.nis());
                }
                j++;
            }

            // Stop once every GPS point has been applied: after that it's IMU-only guessing.
            if (j == gpsptRelative.size()) {
                break;
            }

            if (k % 10 == 0) {
                trajectory.push_back(makeTrajRow(imuInputs[k].t, ekf));
            }
        }
        std::cout << "Used " << j << " of " << gpsptRelative.size() << " GPS points" << std::endl;
        std::cout << "Final state: " << ekf.state().transpose() << std::endl;
        printScores(log, ekf.numRejected(), ekf.numRecoveries());

        // Save results for plotting (python3 tools/plot_trajectory.py)
        // The no-wheel run gets its own files so the plot can compare the two.
        const std::string suffix = useWheel ? "" : "_nowheel";
        writeTrajectory("output/trajectory" + suffix + ".csv", trajectory);
        writeGpsPoints("output/gps_points" + suffix + ".csv", gpsptRelative, gpsStatus);
        std::cout << "Wrote " << trajectory.size() << " rows to output/trajectory" << suffix << ".csv" << std::endl;
    }

    catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << "\n";
        return 1;
    }

    return 0;
}