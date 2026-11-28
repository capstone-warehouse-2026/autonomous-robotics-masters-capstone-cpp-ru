#include "capstone_benchmark/g1_eval.hpp"

#include <algorithm>
#include <charconv>
#include <cmath>
#include <istream>
#include <limits>
#include <stdexcept>

namespace capstone::g1 {
namespace {
constexpr double kPi = 3.14159265358979323846;
constexpr double kDeg = 180.0/kPi;
double wrap(double x) { return std::atan2(std::sin(x), std::cos(x)); }
Pose between(Pose a, Pose b) {
  const double c=std::cos(a.yaw), s=std::sin(a.yaw), x=b.x-a.x, y=b.y-a.y;
  return {c*x+s*y,-s*x+c*y,wrap(b.yaw-a.yaw)};
}
Pose compose(Pose a, Pose b) {
  const double c=std::cos(a.yaw),s=std::sin(a.yaw);
  return {a.x+c*b.x-s*b.y,a.y+s*b.x+c*b.y,wrap(a.yaw+b.yaw)};
}
std::vector<std::string> split(const std::string& line) {
  std::vector<std::string> parts;
  for(std::size_t i=0;;) {
    const auto comma=line.find(',',i);
    parts.push_back(line.substr(i,comma==std::string::npos?comma:comma-i));
    if(comma==std::string::npos) return parts;
    i=comma+1;
  }
}
double number(const std::string& s) {
  double x{};
  const auto r=std::from_chars(s.data(),s.data()+s.size(),x,std::chars_format::general);
  if(r.ec!=std::errc{} || r.ptr!=s.data()+s.size() || !std::isfinite(x) || std::abs(x)>1e6)
    throw std::runtime_error("invalid/nonfinite or out-of-range pose");
  return x;
}
Stamp timestamp(const std::string& s) {
  Stamp x{};
  const auto r=std::from_chars(s.data(),s.data()+s.size(),x);
  if(r.ec!=std::errc{} || r.ptr!=s.data()+s.size() || x<0 ||
     x>std::numeric_limits<Stamp>::max()-kSecond)
    throw std::runtime_error("invalid simulation stamp");
  return x;
}
double p95(std::vector<double> v) {
  std::sort(v.begin(),v.end());
  return v[static_cast<std::size_t>(std::ceil(.95*v.size()))-1];
}
}  // namespace

Trace read_trace(std::istream& in, bool allow_legacy) {
  std::string line;
  auto read_line=[&] {
    if(!std::getline(in,line)) return false;
    if(!line.empty() && line.back()=='\r') line.pop_back();
    return true;
  };
  if(!read_line()) throw std::runtime_error("missing CSV header");
  const bool legacy=line=="stream,sim_stamp_ns,x_m,y_m,yaw_rad";
  if(legacy && !allow_legacy) throw std::runtime_error("CSV has no frames; explicit --legacy-smoke required");
  if(!legacy && line!="stream,sim_stamp_ns,frame_id,x_m,y_m,yaw_rad")
    throw std::runtime_error("unexpected CSV header");
  Trace data;
  for(std::size_t line_number=2;read_line();++line_number) {
    try {
      const auto fields=split(line);
      const auto size=legacy?5u:6u;
      if(fields.size()!=size) throw std::runtime_error("wrong field count");
      const auto& key=fields[0];
      const bool global=key=="truth" || key=="map";
      if(!global && key!="wheel" && key!="fused") throw std::runtime_error("unknown stream");
      if(!legacy && fields[2]!=(global?"map":"odom")) throw std::runtime_error("stream frame mismatch");
      const auto t=timestamp(fields[1]);
      auto& d=data[key];
      if(!d.empty() && t<=d.rbegin()->first) throw std::runtime_error("duplicate or backward stamp within stream");
      d.emplace(t,Pose{number(fields[size-3]),number(fields[size-2]),wrap(number(fields[size-1]))});
    } catch(const std::exception& e) {
      throw std::runtime_error("CSV line "+std::to_string(line_number)+": "+e.what());
    }
  }
  if(in.bad()) throw std::runtime_error("CSV stream read error");
  if(!data.count("truth") || !data.count("map")) throw std::runtime_error("missing truth or map stream");
  return data;
}

std::optional<Pose> sample(const Stream& s, Stamp stamp, Coverage* c) {
  const auto hi=s.lower_bound(stamp);
  if(hi!=s.end() && hi->first==stamp) {
    if(c) ++c->exact;
    return hi->second;
  }
  if(hi==s.begin() || hi==s.end()) {
    if(c) ++c->outside;
    return {};
  }
  const auto lo=std::prev(hi);
  const auto gap=hi->first-lo->first;
  if(gap>kGap) {
    if(c) ++c->gap;
    return {};
  }
  const double f=static_cast<double>(stamp-lo->first)/gap;
  const auto a=lo->second,b=hi->second;
  if(c) {
    ++c->interpolated;
    c->max_bracket=std::max(c->max_bracket,gap);
  }
  return Pose{a.x+f*(b.x-a.x),a.y+f*(b.y-a.y),wrap(a.yaw+f*wrap(b.yaw-a.yaw))};
}

std::vector<Match> pair(const Stream& gt, const Stream& source, Stamp start, Coverage& c) {
  std::vector<Match> out;
  c={};
  for(auto it=source.lower_bound(start);it!=source.end();++it) {
    ++c.requested;
    if(auto g=sample(gt,it->first,&c)) out.push_back({it->first,*g,it->second});
  }
  return out;
}

std::vector<Match> align_first(const std::vector<Match>& rows) {
  auto out=rows;
  if(!rows.empty()) for(auto& a:out)
    a.estimate=compose(rows.front().truth,between(rows.front().estimate,a.estimate));
  return out;
}

Summary summarize(const std::vector<Match>& rows) {
  if(rows.size()<2) throw std::runtime_error("need at least two matched samples");
  Stream gt,est;
  std::vector<double> xy,yaw;
  xy.reserve(rows.size());yaw.reserve(rows.size());
  double sq_xy=0,sq_yaw=0,recovery_sum=0;
  std::size_t recovered=0;
  Summary out;
  std::optional<Stamp> bad_start;
  Stamp prev=rows.front().stamp;
  for(const auto& r:rows) {
    if(!gt.empty() && r.stamp<=prev) throw std::runtime_error("non-monotonic pair rows");
    if(bad_start && r.stamp-prev>kGap) {
      if(prev-*bad_start>kSecond) ++out.loss_censored;
      bad_start.reset();
    }
    gt.emplace(r.stamp,r.truth); est.emplace(r.stamp,r.estimate);
    const auto e=between(r.truth,r.estimate);
    const double d=std::hypot(e.x,e.y), h=std::abs(e.yaw);
    xy.push_back(d); yaw.push_back(h*kDeg);
    sq_xy+=d*d; sq_yaw+=h*h*kDeg*kDeg;
    if(d>.5 || h>15.0/kDeg) {
      if(!bad_start) bad_start=r.stamp;
      const auto elapsed=r.stamp-*bad_start;
      out.longest_loss_s=std::max(out.longest_loss_s,static_cast<double>(elapsed)/kSecond);
    } else if(bad_start) {
      if(prev-*bad_start>kSecond) {
        ++recovered;
        recovery_sum+=static_cast<double>(r.stamp-*bad_start)/kSecond;
      }
      bad_start.reset();
    }
    prev=r.stamp;
  }
  if(bad_start && prev-*bad_start>kSecond) ++out.loss_censored;
  out.loss_episodes=recovered+out.loss_censored;
  if(recovered) out.recovery_mean_s=recovery_sum/recovered;
  out.ate=std::sqrt(sq_xy/rows.size()); out.yaw_rmse=std::sqrt(sq_yaw/rows.size());
  out.p95=p95(xy);out.yaw_p95=p95(yaw);
  double rpe_xy=0,rpe_yaw=0;
  for(const auto& r:rows) {
    const Stamp t=r.stamp+kSecond;
    const auto g=sample(gt,t), e=sample(est,t);
    if(!g || !e) continue;
    const auto rpe=between(between(r.truth,*g),between(r.estimate,*e));
    rpe_xy+=rpe.x*rpe.x+rpe.y*rpe.y;
    rpe_yaw+=rpe.yaw*rpe.yaw;
    ++out.rpe_n;
  }
  if(out.rpe_n) {
    out.rpe_xy=std::sqrt(rpe_xy/out.rpe_n);
    out.rpe_yaw=kDeg*std::sqrt(rpe_yaw/out.rpe_n);
  }
  return out;
}
}  // namespace capstone::g1
