#include "evaluation.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>

std::vector<bool> outageMask(const std::vector<Gpspt>& points) {
    std::vector<bool> outage(points.size(), false);
    if (points.empty()) {
        return outage;
    }
    const double first = points.front().t;
    const double last = points.back().t;
    for (double start = first + 100.0; start < last - 40.0; start += 200.0) {
        // Only use windows with good GPS coverage (> 250 of ~300 fixes),
        // otherwise there's nothing to drop.
        int count = 0;
        for (const auto& p : points) {
            if (p.t >= start && p.t < start + 30.0) {
                count++;
            }
        }
        if (count > 250) {
            for (size_t i = 0; i < points.size(); i++) {
                if (points[i].t >= start && points[i].t < start + 30.0) {
                    outage[i] = true;
                }
            }
        }
    }
    return outage;
}

std::vector<bool> heldOutMask(const std::vector<Gpspt>& points, const std::vector<bool>& outage) {
    std::vector<bool> held(points.size(), false);
    for (size_t i = 5; i < points.size(); i += 10) {
        held[i] = !outage[i];
    }
    return held;
}

// True for samples within `hold` s after a gap longer than `gap` s. Right
// after a GPS gap the filter is still recovering, which isn't what we want
// to score.
static std::vector<bool> afterGap(const std::vector<double>& t, double gap = 5.0, double hold = 5.0) {
    std::vector<bool> mask(t.size(), false);
    for (size_t k = 1; k < t.size(); k++) {
        if (t[k] - t[k - 1] > gap) {
            for (size_t i = k; i < t.size() && t[i] < t[k] + hold; i++) {
                mask[i] = true;
            }
        }
    }
    return mask;
}

static double rms(const std::vector<double>& v) {
    double sum = 0.0;
    for (double x : v) {
        sum += x * x;
    }
    return v.empty() ? 0.0 : std::sqrt(sum / v.size());
}

static double mean(const std::vector<double>& v) {
    double sum = 0.0;
    for (double x : v) {
        sum += x;
    }
    return v.empty() ? 0.0 : sum / v.size();
}

// Linear-interpolated percentile, same as numpy's default.
static double percentile(std::vector<double> v, double p) {
    if (v.empty()) {
        return 0.0;
    }
    std::sort(v.begin(), v.end());
    const double pos = p / 100.0 * (v.size() - 1);
    const size_t lo = static_cast<size_t>(std::floor(pos));
    const size_t hi = std::min(lo + 1, v.size() - 1);
    return v[lo] + (pos - lo) * (v[hi] - v[lo]);
}

void printScores(const RunLog& log, int numRejected, int numRecoveries) {
    const std::vector<bool> fixGap = afterGap(log.fixT);
    const std::vector<bool> heldGap = afterGap(log.heldT);

    std::vector<double> prior;
    std::vector<double> nis;
    for (size_t i = 0; i < log.fixT.size(); i++) {
        if (log.accepted[i] && !fixGap[i]) {
            prior.push_back(log.priorErr[i]);
        }
        if (log.accepted[i]) {
            nis.push_back(log.nis[i]);
        }
    }
    std::vector<double> held;
    for (size_t i = 0; i < log.heldT.size(); i++) {
        if (!heldGap[i]) {
            held.push_back(log.heldErr[i]);
        }
    }

    // The long hole: the fix after the biggest gap between fixes.
    size_t holeEnd = 0;
    double biggestGap = 0.0;
    for (size_t i = 1; i < log.fixT.size(); i++) {
        if (log.fixT[i] - log.fixT[i - 1] > biggestGap) {
            biggestGap = log.fixT[i] - log.fixT[i - 1];
            holeEnd = i;
        }
    }

    std::printf("\n%-22s %10s\n", "Score (m)", "C++");
    std::printf("%-22s %10.2f\n", "prior RMSE", rms(prior));
    std::printf("%-22s %10.2f\n", "held-out RMSE", rms(held));
    std::printf("%-22s %10.2f\n", "held-out p95", percentile(held, 95.0));
    std::printf("%-22s %10.2f\n", "30 s outage median", percentile(log.outageErr, 50.0));
    std::printf("%-22s %10.2f\n", "30 s outage mean", mean(log.outageErr));
    std::printf("%-22s %10.2f\n", "mean NIS (ideal 2)", mean(nis));
    std::printf("%-22s %10.2f   (%.0f s hole)\n", "long-hole miss",
                log.priorErr.empty() ? 0.0 : log.priorErr[holeEnd], biggestGap);
    std::printf("Gated out %d fixes, %d recoveries. Held-out: %zu, outage windows: %zu\n",
                numRejected, numRecoveries, log.heldT.size(), log.outageErr.size());
}
