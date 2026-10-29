#include "output.hpp"

#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <stdexcept>

TrajRow makeTrajRow(double t, const Ekf2D& ekf) {
    const Vec5& x = ekf.state();
    const Mat5& P = ekf.covariance();

    TrajRow row;
    row.t = t;
    row.px = x(0);
    row.py = x(1);
    row.theta = x(2);
    row.v = x(3);
    row.stdPx = std::sqrt(P(0, 0));
    row.stdPy = std::sqrt(P(1, 1));
    return row;
}

// Open path for writing, creating its folder first.
static std::ofstream openCsv(const std::string& path) {
    const std::filesystem::path parent = std::filesystem::path(path).parent_path();
    if (!parent.empty()) {
        std::filesystem::create_directories(parent);
    }
    std::ofstream file(path);
    if (!file.is_open()) {
        throw std::runtime_error("Failed to open file for writing: " + path);
    }
    file << std::fixed << std::setprecision(6);
    return file;
}

void writeTrajectory(const std::string& path, const std::vector<TrajRow>& rows) {
    std::ofstream file = openCsv(path);
    file << "t,px,py,theta,v,std_px,std_py\n";
    for (const auto& r : rows) {
        file << r.t << ',' << r.px << ',' << r.py << ',' << r.theta << ','
             << r.v << ',' << r.stdPx << ',' << r.stdPy << '\n';
    }
}

void writeGpsPoints(const std::string& path, const std::vector<Gpspt>& points,
                    const std::vector<int>& status) {
    std::ofstream file = openCsv(path);
    file << "t,east,north,status\n";
    for (size_t i = 0; i < points.size(); i++) {
        file << points[i].t << ',' << points[i].east << ',' << points[i].north << ','
             << status[i] << '\n';
    }
}
