#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <limits>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <sys/resource.h>

#include "capstone_perception/grid_builder.hpp"
#include "capstone_perception/perception_node.hpp"

namespace {

template<class T>
void read(std::istream &in, T *data, std::size_t count) {
  if (!in.read(reinterpret_cast<char *>(data), count * sizeof(T))) {
    throw std::runtime_error("truncated replay");
  }
}

Eigen::Isometry3d read_pose(std::istream &in) {
  Eigen::Matrix<double, 4, 4, Eigen::RowMajor> matrix;
  read(in, matrix.data(), 16);
  return Eigen::Isometry3d(matrix);
}

int index(int value) {
  if (value == 0) return 0;
  if (value == 100) return 1;
  if (value == -1) return 2;
  throw std::runtime_error("unexpected grid class");
}

double ratio(double numerator, double denominator) {
  return denominator > 0 ? numerator / denominator : std::numeric_limits<double>::quiet_NaN();
}

void run(int argc, char **argv) {
  using namespace capstone_perception;
  const auto args = rclcpp::remove_ros_arguments(argc, argv);
  if (args.size() != 3) {
    throw std::runtime_error("usage: grid_replay FILE.rgbd OUTPUT.csv [--ros-args -p method:=learned]");
  }
  rclcpp::Node node("grid_replay");
  auto classifier = make_classifier(node.declare_parameter<std::string>("method", "learned"), node);
  const bool without_rgb = node.declare_parameter<bool>("without_rgb", false);
  GridBuilder builder(0.05, 10.0, 2, 1, 1);
  std::ifstream input(args[1], std::ios::binary);
  std::array<char, 8> magic{};
  read(input, magic.data(), magic.size());
  if (std::string(magic.data(), magic.size()) != "G2RGBD01") {
    throw std::runtime_error("invalid replay version");
  }
  std::ofstream csv(args[2]);
  csv.exceptions(std::ios::failbit | std::ios::badbit);
  csv << "frame,processing_ms,gt_free_pred_free,gt_free_pred_obstacle,gt_free_pred_unknown,"
         "gt_obstacle_pred_free,gt_obstacle_pred_obstacle,gt_obstacle_pred_unknown,"
         "gt_unknown_pred_free,gt_unknown_pred_obstacle,gt_unknown_pred_unknown\n";
  std::ofstream distance_csv(args[2] + ".distance.csv");
  distance_csv.exceptions(std::ios::failbit | std::ios::badbit);
  distance_csv << "bin_m,gt_obstacle,pred_free,pred_obstacle,pred_unknown\n";
  std::array<std::array<std::uint64_t, 3>, 3> distance_counts{};
  std::array<std::uint64_t, 9> total{};
  std::vector<double> timings;
  while (input.peek() != EOF) {
    std::uint32_t height, width;
    read(input, &height, 1);
    read(input, &width, 1);
    if (height != 480 || width != 640) throw std::runtime_error("expected 640x480 replay");
    std::array<double, 4> k{};
    read(input, k.data(), k.size());
    RgbdFrame frame;
    frame.camera = {k[0], k[1], k[2], k[3], int(width), int(height)};
    frame.base_from_camera = read_pose(input);
    const auto world_from_base = read_pose(input);
    const Eigen::Isometry3d world_from_camera = world_from_base * frame.base_from_camera;
    const auto origin = world_from_base.translation();
    frame.rgb.create(height, width, CV_8UC3);
    frame.depth.create(height, width, CV_32FC1);
    cv::Mat clean(height, width, CV_32FC1), labels(height, width, CV_8UC1);
    read(input, frame.rgb.ptr<std::uint8_t>(), height * width * 3);
    read(input, frame.depth.ptr<float>(), height * width);
    read(input, clean.ptr<float>(), height * width);
    read(input, labels.ptr<std::uint8_t>(), height * width);

    if (without_rgb) frame.rgb.setTo(cv::Scalar::all(0));

    const auto start = std::chrono::steady_clock::now();
    const auto prediction = classifier->classify(frame);
    const auto error = validate_labels(prediction, frame);
    if (!error.empty()) throw std::runtime_error(error);
    const auto result = builder.build(prediction, frame, world_from_camera, origin.x(), origin.y());
    const double ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - start).count();

    frame.depth = clean;
    const auto reference = builder.build(labels, frame, world_from_camera, origin.x(), origin.y());
    std::array<std::uint64_t, 9> counts{};
    for (std::size_t i = 0; i < result.data.size(); ++i) {
      const int expected = index(reference.data[i]);
      const int predicted = index(result.data[i]);
      ++counts[3 * expected + predicted];
      if (expected == 1) {
        const auto column = i % reference.width;
        const auto row = i / reference.width;
        const double x = reference.origin_x + (column + 0.5) * reference.resolution;
        const double y = reference.origin_y + (row + 0.5) * reference.resolution;
        const double metres = std::hypot(x - origin.x(), y - origin.y());
        ++distance_counts[metres < 2.0 ? 0 : metres < 4.0 ? 1 : 2][predicted];
      }
    }
    csv << timings.size() << ',' << ms;
    for (std::size_t i = 0; i < counts.size(); ++i) {
      total[i] += counts[i];
      csv << ',' << counts[i];
    }
    csv << '\n';
    timings.push_back(ms);
  }
  if (timings.empty()) throw std::runtime_error("empty replay");
  csv.close();
  const std::array<const char *, 3> bins{"[0;2)", "[2;4)", "[4;inf)"};
  for (std::size_t i = 0; i < bins.size(); ++i) {
    const auto &counts = distance_counts[i];
    distance_csv << bins[i] << ',' << counts[0] + counts[1] + counts[2]
                 << ',' << counts[0] << ',' << counts[1] << ',' << counts[2] << '\n';
  }
  distance_csv.close();
  std::sort(timings.begin(), timings.end());
  const auto quantile = [&](double fraction) {
    return timings[static_cast<std::size_t>(std::ceil(fraction * timings.size())) - 1];
  };
  const double free_iou = ratio(total[0], total[0] + total[1] + total[2] + total[3]);
  const double obstacle_iou = ratio(total[4], total[4] + total[1] + total[3] + total[5]);
  const auto obstacle = total[3] + total[4] + total[5];
  rusage usage{};
  if (getrusage(RUSAGE_SELF, &usage) != 0) throw std::runtime_error("getrusage failed");
  std::cout << "frames,iou_free,iou_obstacle,macro_iou_valid,obstacle_precision,obstacle_recall,"
               "false_free,p50_ms,p95_ms,p99_ms,peak_rss_kib\n"
            << timings.size() << ',' << free_iou << ',' << obstacle_iou << ',' << (free_iou + obstacle_iou) / 2 << ','
            << ratio(total[4], total[1] + total[4]) << ',' << ratio(total[4], obstacle) << ','
            << ratio(total[3], obstacle) << ',' << quantile(.50) << ',' << quantile(.95) << ','
            << quantile(.99) << ',' << usage.ru_maxrss << '\n';
}

}  // namespace

int main(int argc, char **argv) {
  try {
    rclcpp::init(argc, argv);
    run(argc, argv);
    rclcpp::shutdown();
    return 0;
  } catch (const std::exception &error) {
    std::cerr << "grid_replay: " << error.what() << '\n';
    return 1;
  }
}
