// Method B in C++ against Python: the golden data in test/data is written by ml/perception/release_model.py
// from a test frame and the packaged model with ONNX Runtime in Python. The C++ preprocessing, inference and
// post-processing must reproduce it; otherwise the network in the robot is not the network that was evaluated.

#include <cmath>
#include <fstream>
#include <string>

#include <gtest/gtest.h>
#include <opencv2/core/persistence.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>

#include "capstone_perception/cnn_classifier.hpp"

namespace {

using capstone_perception::CnnClassifier;
using capstone_perception::CnnModel;
using capstone_perception::PixelClass;
using capstone_perception::RgbdFrame;

const std::string kPackage = CAPSTONE_PERCEPTION_SOURCE_DIR;
const std::string kModel = kPackage + "/models/perception_cnn.onnx";
const std::string kData = kPackage + "/test/data/";

// The golden frame: RGB PNG and depth PNG in mm (0 = no measurement), converted as release_model.py does.
RgbdFrame golden_frame() {
  RgbdFrame frame;
  const cv::Mat bgr = cv::imread(kData + "golden_rgb.png", cv::IMREAD_COLOR);
  const cv::Mat millimetres = cv::imread(kData + "golden_depth.png", cv::IMREAD_UNCHANGED);
  if (bgr.empty() || millimetres.type() != CV_16UC1) {
    throw std::runtime_error("golden frame missing in " + kData);
  }
  cv::cvtColor(bgr, frame.rgb, cv::COLOR_BGR2RGB);
  frame.depth.create(millimetres.size(), CV_32FC1);
  for (int v = 0; v < millimetres.rows; ++v) {
    for (int u = 0; u < millimetres.cols; ++u) {
      const auto value = millimetres.at<std::uint16_t>(v, u);
      frame.depth.at<float>(v, u) = value == 0 ? NAN : static_cast<float>(value) / 1000.0f;
    }
  }
  return frame;
}

class CnnClassifierTest : public ::testing::Test {
 protected:
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }

  void SetUp() override {
    cv::FileStorage golden(kData + "golden.yaml", cv::FileStorage::READ);
    ASSERT_TRUE(golden.isOpened());
    stride_ = static_cast<int>(golden["stride"]);
    golden["input"] >> expected_input_;
    golden["logits"] >> expected_logits_;
    frame_ = golden_frame();
  }

  // Value of the full network grid (channels x height x width) at the sampled position (k, i, j).
  static float at(const float *tensor, int height, int width, int k, int row, int column) {
    return tensor[(static_cast<std::size_t>(k) * height + row) * width + column];
  }

  int stride_ = 0;
  cv::Mat expected_input_;   // 5 * rows x columns, every stride-th pixel of the network grid
  cv::Mat expected_logits_;  // 3 * rows x columns
  RgbdFrame frame_;
};

TEST_F(CnnClassifierTest, PreprocessingMatchesPython) {
  CnnModel model(kModel, 1);
  capstone_perception::network_input(frame_, model.height(), model.width(), model.input());
  const int rows = expected_input_.rows / 5;
  ASSERT_EQ(rows, (model.height() + stride_ - 1) / stride_);
  ASSERT_EQ(expected_input_.cols, (model.width() + stride_ - 1) / stride_);
  for (int k = 0; k < 5; ++k) {
    for (int i = 0; i < rows; ++i) {
      for (int j = 0; j < expected_input_.cols; ++j) {
        // Same float operations as numpy: equal up to the last bit.
        ASSERT_EQ(at(model.input(), model.height(), model.width(), k, i * stride_, j * stride_),
                  expected_input_.at<float>(k * rows + i, j))
            << "channel " << k << ", row " << i * stride_ << ", column " << j * stride_;
      }
    }
  }
}

TEST_F(CnnClassifierTest, LogitsMatchPythonOnnxRuntime) {
  CnnModel model(kModel, 1);
  capstone_perception::network_input(frame_, model.height(), model.width(), model.input());
  const float *logits = model.run();
  const int rows = expected_logits_.rows / 3;
  for (int k = 0; k < 3; ++k) {
    for (int i = 0; i < rows; ++i) {
      for (int j = 0; j < expected_logits_.cols; ++j) {
        // The same ONNX Runtime version; CPU kernels may differ slightly between machines.
        ASSERT_NEAR(at(logits, model.height(), model.width(), k, i * stride_, j * stride_),
                    expected_logits_.at<float>(k * rows + i, j), 1e-3)
            << "class " << k << ", row " << i * stride_ << ", column " << j * stride_;
      }
    }
  }
}

TEST_F(CnnClassifierTest, LabelsMatchPythonAndKeepContract) {
  rclcpp::Node node("test_cnn", rclcpp::NodeOptions().parameter_overrides({{"learned.model_path", kModel}}));
  CnnClassifier classifier(node);
  const cv::Mat labels = classifier.classify(frame_);
  const cv::Mat expected = cv::imread(kData + "golden_labels.png", cv::IMREAD_UNCHANGED);
  ASSERT_EQ(expected.type(), CV_8UC1);
  EXPECT_EQ(capstone_perception::validate_labels(labels, frame_), "");
  // Arg-max may flip only where two logits are nearly equal; allow 0.1 % of the pixels.
  EXPECT_LE(cv::countNonZero(labels != expected), labels.total() / 1000);
  EXPECT_GT(cv::countNonZero(labels == static_cast<int>(PixelClass::kObstacle)), 0);
  EXPECT_GT(cv::countNonZero(labels == static_cast<int>(PixelClass::kFree)), 0);
}

TEST_F(CnnClassifierTest, MissingModelThrowsOnClassifyInsteadOfStoppingTheNode) {
  rclcpp::Node node("test_cnn", rclcpp::NodeOptions().parameter_overrides(
                                    {{"learned.model_path", std::string("/nonexistent/model.onnx")}}));
  CnnClassifier classifier(node);  // must not throw: the node falls back to the classical method
  EXPECT_THROW(classifier.classify(frame_), std::runtime_error);
}

TEST_F(CnnClassifierTest, BrokenModelThrowsOnClassify) {
  const std::string path = ::testing::TempDir() + "broken_model.onnx";
  std::ofstream(path) << "not an ONNX model";
  rclcpp::Node node("test_cnn", rclcpp::NodeOptions().parameter_overrides({{"learned.model_path", path}}));
  CnnClassifier classifier(node);
  EXPECT_THROW(classifier.classify(frame_), std::runtime_error);
}

TEST_F(CnnClassifierTest, RejectsFramesOfAnotherSize) {
  rclcpp::Node node("test_cnn", rclcpp::NodeOptions().parameter_overrides({{"learned.model_path", kModel}}));
  CnnClassifier classifier(node);
  RgbdFrame small;
  small.rgb = cv::Mat::zeros(240, 320, CV_8UC3);
  small.depth = cv::Mat(240, 320, CV_32FC1, cv::Scalar(1.0));
  EXPECT_THROW(classifier.classify(small), std::runtime_error);
}

TEST(CnnPostprocessing, ArgmaxUpsamplingAndNanAreUnknown) {
  const int height = 1;
  const int width = 3;
  // Logits of classes free, obstacle, unknown for three network pixels; the last one is a tie.
  const float logits[] = {0.0f, 0.0f, 1.0f,  // free
                          1.0f, 0.0f, 1.0f,  // obstacle
                          0.0f, 2.0f, 0.0f};  // unknown
  cv::Mat depth(2, 6, CV_32FC1, cv::Scalar(1.0));
  depth.at<float>(1, 1) = NAN;
  const cv::Mat labels = capstone_perception::labels_from_logits(logits, height, width, depth);
  const cv::Mat expected = (cv::Mat_<std::uint8_t>(2, 6) << 1, 1, 255, 255, 0, 0,  //
                            1, 255, 255, 255, 0, 0);
  EXPECT_EQ(cv::countNonZero(labels != expected), 0);
}

TEST(CnnPostprocessing, NonFiniteLogitsThrow) {
  const float logits[] = {0.0f, NAN, 0.0f};
  const cv::Mat depth(2, 2, CV_32FC1, cv::Scalar(1.0));
  EXPECT_THROW(capstone_perception::labels_from_logits(logits, 1, 1, depth), std::runtime_error);
}

}  // namespace
