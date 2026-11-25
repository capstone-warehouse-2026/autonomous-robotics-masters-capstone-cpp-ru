#include <gtest/gtest.h>
#include <unistd.h>

#include <Eigen/LU>
#include <fstream>
#include <limits>

#include "capstone_localization/localization.hpp"
using namespace capstone_localization;
TEST(Geometry, WrapAndComposition) {
    EXPECT_NEAR(wrap_angle(3.141592653589793 + 0.1), -3.141592653589793 + 0.1, 1e-12);
    Pose a(1, 2, 0.8), b(-2, 3, -0.5);
    EXPECT_LT((compose(a, between(a, b)) - b).norm(), 1e-12);
}
TEST(Filter, PositiveCovarianceAndWrappedInnovation) {
    Filter f;
    f.pose.z() = 3.13;
    f.covariance = Covariance::Identity();
    Pose z(0, 0, -3.13);
    Covariance r = Covariance::Identity() * 0.01;
    EXPECT_LT(f.nis(z, r), 0.001);
    f.correct(z, r);
    EXPECT_GT(f.covariance.determinant(), 0);
    EXPECT_TRUE(f.covariance.allFinite());
}
TEST(Localization, RejectsInvalidInputsAndDuplicateTimes) {
    Localization l;
    EXPECT_FALSE(l.wheel(NAN, 1, 0, {}));
    EXPECT_TRUE(l.wheel(1, 1, 0, {}));
    EXPECT_FALSE(l.wheel(1, 1, 0, {}));
    EXPECT_FALSE(l.wheel(.9, 1, 0, {}));
    EXPECT_TRUE(l.wheel(1.1, 1, 0, {}));
    EXPECT_NEAR(l.estimate()->odom.pose.x(), .1, 1e-10);
}
TEST(Localization, ImuFallbackAndReset) {
    Localization l;
    l.wheel(10, 1, 0, {});
    l.wheel(10.1, 1, 0, 1.0);
    EXPECT_TRUE(l.estimate()->imu_used);
    EXPECT_NEAR(l.estimate()->global.pose.z(), .1, 1e-10);
    l.wheel(10.2, 1, 0, NAN);
    EXPECT_FALSE(l.estimate()->imu_used);
    l.wheel(0, 0, 0, {});
    EXPECT_NEAR(l.estimate()->global.pose.norm(), 0, 1e-10);
}
TEST(Localization, DropoutDoesNotInventMotion) {
    Localization l;
    l.wheel(0, 1, 0, {});
    l.wheel(2, 1, 0, {});
    EXPECT_NEAR(l.estimate()->global.pose.x(), 0, 1e-10);
    EXPECT_GT(l.estimate()->global.covariance(0, 0), 1);
}
TEST(Localization, RejectsStaleScansAndMissingMap) {
    Localization l;
    l.wheel(5, 0, 0, {});
    EXPECT_EQ(l.scan(4, {}).reason, "stale_or_unsynchronized_scan");
    EXPECT_EQ(l.scan(6, {}).reason, "stale_or_unsynchronized_scan");
    EXPECT_EQ(l.scan(5, {}).reason, "no_map");
}
TEST(Localization, MissingModelHasExplicitFallback) {
    Localization l({}, true, "/nonexistent/g1.model");
    EXPECT_FALSE(l.model_error().empty());
    l.wheel(0, 0, 0, {});
    EXPECT_TRUE(l.scan(0, {}).fallback);
}
TEST(Localization, InvalidConfigRejected) {
    LocalizationConfig c;
    c.fixed_variance.x() = -1;
    EXPECT_THROW(Localization l(c), std::invalid_argument);
}
TEST(ScanMatcher, RejectsOutliersAndEmptyMap) {
    ScanMatcher matcher;
    EXPECT_FALSE(matcher.match({}, Pose::Zero(), .2));
    std::vector<std::int8_t> map(400, 0);
    for (int i = 0; i < 20; ++i)
        map[i] = 100;
    matcher.set_map(20, 20, .1, Pose::Zero(), map);
    Points distant(100, Eigen::Vector2d(100, 100));
    EXPECT_FALSE(matcher.match(distant, Pose::Zero(), .2));
}

TEST(ScanMatcher, RecoversPoseInRoom) {
    std::vector<std::int8_t> map(600, 0);
    for (int y = 0; y < 20; ++y)
        for (int x = 0; x < 30; ++x)
            if (x == 0 || y == 0 || x == 29 || y == 19)
                map[y * 30 + x] = 100;
    ScanMatcher matcher;
    matcher.set_map(30, 20, .1, Pose::Zero(), map);
    Points points;
    for (int i = 0; i < 360; ++i) {
        double a = i * 6.283185307179586 / 360, c = std::cos(a), s = std::sin(a);
        double t = std::min(std::abs(c) > 1e-8 ? 1.4 / std::abs(c) : 1e9,
                            std::abs(s) > 1e-8 ? .9 / std::abs(s) : 1e9);
        points.emplace_back(t * c, t * s);
    }
    auto match = matcher.match(points, Pose(1.54, 1.03, .02), .22);
    ASSERT_TRUE(match);
    EXPECT_LT((match->pose - Pose(1.5, 1, 0)).norm(), .02);
    LocalizationConfig config;
    config.initial = Pose(1.54, 1.03, .02);
    Localization localization(config);
    localization.set_map(30, 20, .1, Pose::Zero(), map);
    localization.wheel(0, 0, 0, {});
    auto update = localization.scan(0, points);
    EXPECT_TRUE(update.accepted) << update.reason;
    EXPECT_EQ(localization.scan(0, points).reason, "stale_or_unsynchronized_scan");
}

TEST(Localization, NonFiniteModelIsRejected) {
    const std::string path = "/tmp/g1-invalid-model-" + std::to_string(getpid());
    {
        std::ofstream out(path);
        out << "G1_MLP_V1 nan";
    }
    Localization l({}, true, path);
    std::remove(path.c_str());
    EXPECT_FALSE(l.model_error().empty());
    l.wheel(0, 0, 0, {});
    EXPECT_TRUE(l.scan(0, {}).fallback);
}
