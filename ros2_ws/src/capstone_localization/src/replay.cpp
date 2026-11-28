// Replays exported sensor-only events. Ground truth is deliberately not an input.
#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <stdexcept>

#include "capstone_localization/localization.hpp"
using namespace capstone_localization;
int main(int argc, char **argv) {
    try {
        if (argc != 10)
            throw std::runtime_error(
                "usage: localization_replay MAP EVENTS OUTPUT classical|learned MODEL X Y YAW "
                "COVARIANCE_SCALE");
        std::string method = argv[4];
        if (method != "classical" && method != "learned")
            throw std::runtime_error("invalid method");
        LocalizationConfig config;
        config.initial = Pose(std::stod(argv[6]), std::stod(argv[7]), std::stod(argv[8]));
        // Last argument freezes scalar covariance multiplier, selected on validation.
        double scale = std::stod(argv[9]);
        if (!std::isfinite(scale) || scale <= 0)
            throw std::runtime_error("invalid covariance scale");
        config.fixed_variance *= scale;
        Localization filter(config, method == "learned", argv[5]);
        std::ifstream map(argv[1]), events(argv[2]);
        std::ofstream out(argv[3]);
        unsigned w, h;
        double resolution;
        Pose origin;
        if (!(map >> w >> h >> resolution >> origin.x() >> origin.y() >> origin.z()) || w == 0 ||
            h == 0 || uint64_t(w) * h > 100000000)
            throw std::runtime_error("invalid map");
        std::vector<int8_t> cells(size_t(w) * h);
        for (auto &c : cells) {
            int v;
            if (!(map >> v) || v < -1 || v > 100)
                throw std::runtime_error("invalid map cells");
            c = v;
        }
        filter.set_map(w, h, resolution, origin, cells);
        if (!events || !out)
            throw std::runtime_error("cannot open events/output");
        out << std::setprecision(17)
            << "kind,stamp,x,y,yaw,p00,p01,p02,p10,p11,p12,p20,p21,p22,accepted,fallback,nis,"
               "latency_us,mx,my,myaw,f0,f1,f2,f3,f4,f5,f6,rx,ry,ryaw,reason\n";
        std::string line;
        size_t count = 0;
        while (std::getline(events, line)) {
            if (line.empty())
                continue;
            std::istringstream in(line);
            char kind;
            double stamp;
            if (!(in >> kind >> stamp))
                throw std::runtime_error("invalid event");
            auto start = std::chrono::steady_clock::now();
            ScanUpdate update;
            if (kind == 'W') {
                double v, yaw, rate;
                int valid;
                if (!(in >> v >> yaw >> valid >> rate))
                    throw std::runtime_error("invalid wheel event");
                if (!filter.wheel(stamp, v, yaw,
                                  valid ? std::optional<double>(rate) : std::nullopt))
                    continue;
                update.reason = "prediction";
            } else if (kind == 'S') {
                size_t n;
                if (!(in >> n) || n > 100000)
                    throw std::runtime_error("invalid scan length");
                Points points(n);
                for (auto &p : points)
                    if (!(in >> p.x() >> p.y()))
                        throw std::runtime_error("invalid scan point");
                update = filter.scan(stamp, points);
            } else
                throw std::runtime_error("unknown event type");
            const double us =
                std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - start)
                    .count();
            if (!filter.estimate())
                continue;
            const auto &f = filter.estimate()->global;
            out << kind << ',' << stamp;
            for (double v : f.pose)
                out << ',' << v;
            for (int i = 0; i < 3; ++i)
                for (int j = 0; j < 3; ++j)
                    out << ',' << f.covariance(i, j);
            out << ',' << update.accepted << ',' << update.fallback << ',' << update.nis << ','
                << us;
            Pose match = update.match ? update.match->pose
                                      : Pose::Constant(std::numeric_limits<double>::quiet_NaN());
            for (double v : match)
                out << ',' << v;
            for (float v : update.features)
                out << ',' << v;
            for (double v : update.measurement_variance)
                out << ',' << v;
            out << ',' << update.reason << '\n';
            ++count;
        }
        if (!count)
            throw std::runtime_error("empty replay");
        std::cerr << count << " rows; model status: "
                  << (filter.model_error().empty() ? "ok" : filter.model_error()) << '\n';
        return 0;
    } catch (const std::exception &e) {
        std::cerr << e.what() << '\n';
        return 1;
    }
}
