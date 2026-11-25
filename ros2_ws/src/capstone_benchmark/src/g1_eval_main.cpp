#include "capstone_benchmark/g1_eval.hpp"

#include <charconv>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <tuple>

using namespace capstone::g1;
namespace {
void opt(std::ostream& s, const std::optional<double> v) {
  if(v) s<<*v; else s<<"null";
}
void write(std::ostream& s, const Summary& m) {
  s<<"{\"ate_xy_rmse_m\":"<<m.ate<<",\"xy_p95_m\":"<<m.p95
   <<",\"yaw_rmse_deg\":"<<m.yaw_rmse<<",\"yaw_p95_deg\":"<<m.yaw_p95
   <<",\"rpe_1s_n\":"<<m.rpe_n<<",\"rpe_1s_xy_rmse_m\":";
  opt(s,m.rpe_xy);s<<",\"rpe_1s_yaw_rmse_deg\":";opt(s,m.rpe_yaw);
  s<<",\"loss_window_max_s\":"<<m.longest_loss_s<<",\"loss_episodes\":"<<m.loss_episodes
   <<",\"loss_right_censored\":"<<m.loss_censored<<",\"recovery_mean_s\":";
  opt(s,m.recovery_mean_s);s<<'}';
}
void coverage(std::ostream& s, const Coverage& c) {
  s<<"{\"requested\":"<<c.requested<<",\"paired\":"<<c.exact+c.interpolated
   <<",\"exact\":"<<c.exact<<",\"interpolated\":"<<c.interpolated
   <<",\"outside_truth_span\":"<<c.outside<<",\"truth_gap_too_large\":"<<c.gap
   <<",\"max_used_bracket_ms\":"<<c.max_bracket/1e6<<'}';
}
}  // namespace

int main(int argc, char** argv) {
  try {
    if(argc<5 || argc>6) throw std::runtime_error(
      "usage: g1_trajectory_eval INPUT.csv METRICS.json PAIRED.csv START_NS [--legacy-smoke]");
    const std::string start_arg(argv[4]);
    Stamp start{};
    const auto parsed=std::from_chars(start_arg.data(),start_arg.data()+start_arg.size(),start);
    if(parsed.ec!=std::errc{} || parsed.ptr!=start_arg.data()+start_arg.size() || start<0 ||
       start>std::numeric_limits<Stamp>::max()-kSecond)
      throw std::runtime_error("invalid START_NS");
    if(argc==6 && std::string(argv[5])!="--legacy-smoke") throw std::runtime_error("unknown option");
    if(std::string(argv[1])==argv[2] || std::string(argv[1])==argv[3] || std::string(argv[2])==argv[3])
      throw std::runtime_error("input and output arguments must be different");
    std::ifstream in(argv[1]);
    if(!in) throw std::runtime_error("cannot open input");
    const auto trace=read_trace(in,argc==6);
    std::ofstream json(argv[2]),csv(argv[3]);
    if(!json || !csv) throw std::runtime_error("cannot open outputs");
    json<<std::setprecision(17); csv<<std::setprecision(17);
    csv<<"stream,sim_stamp_ns,gt_x_m,gt_y_m,gt_yaw_rad,estimate_x_m,estimate_y_m,estimate_yaw_rad\n";
    json<<"{\n\"protocol\":\"g1_course_offline_v1\",\"start_ns\":"<<start
        <<",\"legacy_smoke_allowed\":"<<(argc==6?"true":"false")
        <<",\"max_bracket_ms\":"<<kGap/1e6<<",\n\"streams\":{";
    bool first=true;
    std::size_t map_n=0;
    for(const auto* key:{"map","wheel","fused"}) {
      const auto it=trace.find(key);
      if(it==trace.end()) continue;
      Coverage c;
      const auto rows=pair(trace.at("truth"),it->second,start,c);
      if(std::string(key)=="map") map_n=rows.size();
      for(const auto& row:rows) {
        csv<<key<<','<<row.stamp<<','<<row.truth.x<<','<<row.truth.y<<','<<row.truth.yaw
           <<','<<row.estimate.x<<','<<row.estimate.y<<','<<row.estimate.yaw<<'\n';
      }
      if(!first) json<<',';
      first=false;
      json<<"\n\""<<key<<"\":{\"coverage\":";
      coverage(json,c);
      if(rows.size()>=2) {
        if(std::string(key)=="map") {
          json<<",\"raw\":";write(json,summarize(rows));
        }
        json<<",\"start_aligned\":";write(json,summarize(align_first(rows)));
      } else {
        if(std::string(key)=="map") json<<",\"raw\":null";
        json<<",\"start_aligned\":null";
      }
      json<<'}';
    }
    json<<"\n}}\n"; json.flush();csv.flush();
    if(!json || !csv) throw std::runtime_error("cannot write outputs");
    if(map_n<2) {
      std::cerr<<"insufficient common-map pairs; coverage saved, metrics unavailable\n";
      return 2;
    }
    std::cout<<"G1 offline C++: "<<map_n<<" common-map pairs; wrote metrics and paired trajectory\n";
    return 0;
  } catch(const std::exception& e) {
    std::cerr<<"g1_trajectory_eval: "<<e.what()<<'\n';
    return 1;
  }
}
