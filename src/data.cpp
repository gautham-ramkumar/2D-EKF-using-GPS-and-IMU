#include "data.hpp"
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <iomanip>
#include <cmath>

std::ostream& operator<<(std::ostream& os, const ImuSample& s) {
    os << std::fixed << std::setprecision(6) << "t: " << s.t << ", ax: " << s.ax << ", ay: " << s.ay << ", az: " << s.az
       << ", gx: " << s.gx << ", gy: " << s.gy << ", gz: " << s.gz;
    return os;
}

std::ostream& operator<<(std::ostream& os, const GpsFix& g) {
    os << std::fixed << std::setprecision(6) << "t: " << g.t << ", lat: " << g.lat << ", lon: " << g.lon << ", alt: " << g.alt;
    return os;
}

// Reads the next comma-separated field from ss and converts it to double.
static double nextField(std::stringstream& ss) {
    std::string field;
    std::getline(ss, field, ',');
    return std::stod(field);
}

std::vector<ImuSample> loadImu(const std::string& path) {
    std::vector<ImuSample> samples;

    std::ifstream file(path);
    if (!file.is_open()) {
        throw std::runtime_error("Failed to open file: " + path);
    }
    std::string line;
    std::getline(file, line); // read and discard the header line

    while (std::getline(file, line)) {
        std::stringstream ss(line);
        ImuSample sample;
        sample.t = nextField(ss);
        sample.ax = nextField(ss);
        sample.ay = nextField(ss);
        sample.az = nextField(ss);
        sample.gx = nextField(ss);
        sample.gy = nextField(ss);
        sample.gz = nextField(ss);
        samples.push_back(sample);
    }

    return samples;
}

std::vector<GpsFix> loadGps(const std::string& path) {
    std::vector<GpsFix> fixes;

    std::ifstream file(path);
    if (!file.is_open()) {
        throw std::runtime_error("Failed to open file: " + path);
    }
    std::string line;
    std::getline(file, line); // read and discard the header line

    while (std::getline(file, line)) {
        std::stringstream ss(line);
        GpsFix fix;
        fix.t = nextField(ss);
        fix.lat = nextField(ss);
        fix.lon = nextField(ss);
        fix.alt = nextField(ss);
        fixes.push_back(fix);
    }

    return fixes;
}

GpsList removeNaNFixes(const GpsList& fixes) {
    GpsList validfixes;
    for (const auto& fix : fixes) {
        if (!std::isnan(fix.lat) && !std::isnan(fix.lon) && !std::isnan(fix.alt)) {
            validfixes.push_back(fix);
        }
    }
    return validfixes;
}

std::ostream& operator<<(std::ostream& os, const ImuInput& i) {
    os << std::fixed << std::setprecision(6) << "t: " << i.t << ", ax: " << i.ax << ", omega: " << i.omega;
    return os;
}

ImuInputList prepareImu(const ImuList& raw) {
    // ---- Loop 1: measure bias over the first 2 seconds ----
    double sumA = 0.0;
    double sumOmega = 0.0;
    int count = 0;

    for (const auto& sample : raw) {
        if (sample.t - raw[0].t >= 2.0) {
            break;
        }
        sumA += sample.ax;
        sumOmega += -sample.gz;   // note the minus: the sign flip
        count++;
    }

    double biasA = sumA / count;
    double biasOmega = sumOmega / count;

    // ---- Loop 2: build the corrected list ----
    ImuInputList inputs;
    for (const auto& sample : raw) {
        ImuInput in;
        in.t = sample.t;
        in.ax = sample.ax - biasA;
        in.omega = -sample.gz - biasOmega;   // flipped gz minus biasOmega
        inputs.push_back(in);
    }
    return inputs;
}
std::vector<WheelSpeed> loadWheel(const std::string& path) {
    std::vector<WheelSpeed> speeds;

    std::ifstream file(path);
    if (!file.is_open()) {
        throw std::runtime_error("Failed to open file: " + path);
    }
    std::string line;
    std::getline(file, line); // read and discard the header line

    while (std::getline(file, line)) {
        std::stringstream ss(line);
        WheelSpeed s;
        s.t = nextField(ss);
        s.v = nextField(ss);   // the third column (yaw rate) isn't used
        speeds.push_back(s);
    }

    return speeds;
}
