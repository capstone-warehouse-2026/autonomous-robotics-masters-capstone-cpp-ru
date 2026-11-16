// Method B in C++ against Python: test/data/ppo_golden.txt is written by ml/control/export_onnx.py from the
// packaged model with ONNX Runtime in Python. The C++ inference must reproduce it; otherwise the policy in
// the robot is not the policy that was evaluated. Without a packaged model these tests are skipped.

#include <cmath>
#include <fstream>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <gtest/gtest.h>
#include "capstone_control/command_limiter.hpp"
#include "capstone_control/ppo_controller.hpp"

namespace {

namespace ppo = capstone_control::ppo;
using namespace capstone_control;

const std::string kPackage = CAPSTONE_CONTROL_SOURCE_DIR;
const std::string kModel = kPackage + "/models/ppo_policy.onnx";
const std::string kGolden = kPackage + "/test/data/ppo_golden.txt";

bool exists(const std::string &path) { return std::ifstream(path).good(); }

// One golden case per line: kObsSize observation values, then kActionSize network outputs.
struct GoldenCase {
  std::vector<float> observation;
  std::vector<float> action;
};

std::vector<GoldenCase> read_golden() {
  std::ifstream file(kGolden);
  std::vector<GoldenCase> cases;
  std::string line;
  while (std::getline(file, line)) {
    if (line.empty() || line[0] == '#') continue;
    std::istringstream values(line);
    GoldenCase c;
    c.observation.resize(ppo::kObsSize);
    c.action.resize(ppo::kActionSize);
    for (auto &x : c.observation) values >> x;
    for (auto &x : c.action) values >> x;
    if (!values) throw std::runtime_error("broken golden line: " + line);
    cases.push_back(c);
  }
  return cases;
}

TEST(PpoPolicyTest, MissingModelThrows) {
  EXPECT_THROW(PpoPolicy(kPackage + "/models/does_not_exist.onnx"), std::runtime_error);
}

TEST(PpoControllerTest, RejectsNullPolicy) {
  EXPECT_THROW(PpoController(nullptr), std::invalid_argument);
}

TEST(PpoPolicyTest, ReproducesPythonOnGoldenCases) {
  if (!exists(kModel) || !exists(kGolden)) GTEST_SKIP() << "no packaged model yet: run export_onnx.py";
  PpoPolicy policy(kModel);
  const auto cases = read_golden();
  ASSERT_GE(cases.size(), 10u);
  for (std::size_t i = 0; i < cases.size(); ++i) {
    const auto a = policy.run(cases[i].observation);
    for (int k = 0; k < ppo::kActionSize; ++k) EXPECT_NEAR(a[k], cases[i].action[k], 1e-5) << "case " << i;
  }
}

TEST(PpoPolicyTest, RejectsWrongObservationSize) {
  if (!exists(kModel)) GTEST_SKIP() << "no packaged model yet";
  PpoPolicy policy(kModel);
  EXPECT_THROW(policy.run(std::vector<float>(ppo::kObsSize - 1)), std::invalid_argument);
}

TEST(PpoControllerTest, CommandWithinTheBoxesAndNoneWithoutPath) {
  if (!exists(kModel)) GTEST_SKIP() << "no packaged model yet";
  PpoController controller(std::make_shared<PpoPolicy>(kModel));
  Path path;
  for (int i = 0; i <= 30; ++i) path.push_back({0.1 * i, 0.0, 0.0});
  PathTracker tracker(path, 1.5);
  ControlInput in;
  in.pose = Pose2D{0.0, 0.0, 0.0};
  tracker.update(in.pose);
  in.path = &tracker;
  in.velocity = Command{0.3, 0.1};
  const auto c = controller.compute(in);
  ASSERT_TRUE(c.has_value());
  EXPECT_GE(c->v, 0.0);
  EXPECT_LE(c->v, ppo::kVMax);
  EXPECT_LE(std::fabs(c->w), ppo::kWMax);

  in.path = nullptr;
  EXPECT_FALSE(controller.compute(in).has_value());
}

}  // namespace
