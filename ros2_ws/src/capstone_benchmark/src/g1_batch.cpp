#include "capstone_benchmark/g1_batch.hpp"
#include <algorithm>
#include <charconv>
#include <cmath>
#include <fstream>
#include <map>
#include <numeric>
#include <set>
#include <stdexcept>
#include <tuple>
#include <type_traits>

namespace capstone::g1 {
namespace {
std::vector<std::string> split(std::string s) {
  if(!s.empty() && s.back()=='\r')s.pop_back();
  std::vector<std::string> x;
  for(std::size_t p=0;;) {
    auto q=s.find(',',p);x.push_back(s.substr(p,q==std::string::npos?q:q-p));
    if(q==std::string::npos)return x;
    p=q+1;
  }
}
template<class T> T natural(const std::string& s) {
  T x{};
  auto p=std::from_chars(s.data(),s.data()+s.size(),x);
  if(p.ec!=std::errc{} || p.ptr!=s.data()+s.size() )
    throw std::runtime_error("invalid unsigned value");
  if constexpr(std::is_signed_v<T>)if(x<0)throw std::runtime_error("invalid unsigned value");
  return x;
}
bool token(const std::string& s) {
  return !s.empty() && s.find_first_not_of("ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789_.-")==std::string::npos;
}
bool hash(const std::string& s) {
  return s.size()==64 && s.find_first_not_of("0123456789abcdef")==std::string::npos;
}
std::string csv_quote(const std::string& s) {
  std::string out="\"";
  for(char c:s) { if(c=='"')out+='"';out+=c; }
  return out+'"';
}
void opt(std::ostream& out,const std::optional<double>& v){if(v)out<<*v;}
}

std::vector<Plan> read_plan(std::istream& in) {
  std::string line;
  const std::string header="episode_id,method,scenario_id,layout_id,scenario_seed,training_seed,evidence,input_id,terminal,start_ns,format,trace_sha256,trace_path";
  if(!std::getline(in,line))throw std::runtime_error("missing matrix header");
  if(!line.empty()&&line.back()=='\r')line.pop_back();
  if(line!=header)throw std::runtime_error("unexpected matrix header");
  std::vector<Plan> out;
  std::set<std::pair<std::string,std::string>> ids;
  std::optional<unsigned> b_training_seed;
  for(std::size_t n=2;std::getline(in,line);++n) {
    try{
      const auto f=split(line);
      if(f.size()!=13)throw std::runtime_error("expected 13 fields");
      for(auto i:{0,1,2,3,5,6,7,8,10})if(!token(f[i]))throw std::runtime_error("invalid identifier");
      if(f[6]!="smoke"&&f[6]!="fixture"&&f[6]!="test")throw std::runtime_error("unknown evidence class");
      if((f[6]=="fixture")!=(f[1]=="fixture") || (f[1]!="A"&&f[1]!="B"&&f[1]!="fixture"))
        throw std::runtime_error("invalid method/evidence combination");
      if(f[6]=="test" && f[1]=="B" && f[5]=="na")throw std::runtime_error("B test requires training seed");
      if(f[5]!="na") {
        const auto train=natural<unsigned>(f[5]);
        if(f[6]=="test"&&f[1]=="B") {
          if(b_training_seed&&*b_training_seed!=train)
            throw std::runtime_error("analyze distinct B training seeds in separate matrices");
          b_training_seed=train;
        }
      }
      if(f[8]!="ready"&&f[8]!="missing"&&f[8]!="timeout"&&f[8]!="crash")throw std::runtime_error("invalid terminal");
      if(f[10]!="framed"&&f[10]!="legacy_smoke")throw std::runtime_error("unknown trace format");
      if(f[6]=="test"&&f[10]!="framed")throw std::runtime_error("test trace must be frame-tagged");
      if(!hash(f[7]))throw std::runtime_error("expected replay input SHA256 identifier");
      if(f[8]=="ready"&&(!hash(f[11])||f[12].empty()||f[12]=="-"))throw std::runtime_error("ready requires trace hash/path");
      if(!ids.emplace(f[0],f[1]).second)throw std::runtime_error("duplicate episode/method");
      std::filesystem::path path=f[12];
      if(path.is_absolute())throw std::runtime_error("trace path must be relative to matrix");
      for(const auto& part:path)if(part=="..")throw std::runtime_error("trace path crosses matrix directory");
      out.push_back({f[0],f[1],f[2],f[3],f[5],f[6],f[7],f[8],f[10],f[11],
                    natural<unsigned>(f[4]),natural<Stamp>(f[9]),path});
    } catch(const std::exception& e){
      throw std::runtime_error("matrix line "+std::to_string(n)+": "+e.what());
    }
  }
  if(in.bad()||out.empty())throw std::runtime_error("matrix read failed or has no planned rows");
  return out;
}

Episode score_episode(Plan plan,const std::filesystem::path& root) {
  Episode e{std::move(plan),"","",{},std::nullopt,{}};
  if(e.plan.terminal!="ready"){e.status=e.plan.terminal;e.reason="declared terminal outcome";return e;}
  std::ifstream f(root/e.plan.trace);
  if(!f){e.status="missing";e.reason="trace file unavailable";return e;}
  try{
    const auto t=read_trace(f,e.plan.format=="legacy_smoke");
    e.truth=t.at("truth");
    const auto rows=pair(e.truth,t.at("map"),e.plan.start,e.coverage);
    const auto total=e.coverage.requested;
    if(rows.size()<2 || rows.size()<.9*total){
      e.status="insufficient";e.reason="need >=2 pairs and >=90% matching coverage";return e;
    }
    e.summary=summarize(rows);e.status="ok";
  }catch(const std::exception& ex){e.status="invalid";e.reason=ex.what();}
  return e;
}

Dist describe(std::vector<double> v) {
  if(v.empty())throw std::runtime_error("no episodes");
  std::sort(v.begin(),v.end());
  const double mean=std::accumulate(v.begin(),v.end(),0.)/v.size();
  double sum=0;for(double x:v)sum+=(x-mean)*(x-mean);
  const auto n=v.size();
  return {mean,(v[(n-1)/2]+v[n/2])/2,n>1?std::sqrt(sum/(n-1)):0.};
}

bool compatible_pair(const Episode& a,const Episode& b) {
  const auto& x=a.plan; const auto& y=b.plan;
  if(x.episode!=y.episode || x.evidence!="test" || y.evidence!="test" ||
     x.method!="A" || y.method!="B" || x.layout!=y.layout || x.scenario!=y.scenario ||
     x.scenario_seed!=y.scenario_seed || x.input_id!=y.input_id || x.start!=y.start ||
     a.truth.size()!=b.truth.size())return false;
  auto j=b.truth.begin();
  for(const auto& [t,p]:a.truth){
    const auto& q=j->second;
    if(t!=j->first || p.x!=q.x || p.y!=q.y || p.yaw!=q.yaw)return false;
    ++j;
  }
  return true;
}

std::pair<double,double> bootstrap_mean_ci(const std::vector<double>& v,std::mt19937& rng) {
  if(v.size()<2)throw std::runtime_error("bootstrap needs >=2 episodes");
  std::uniform_int_distribution<std::size_t> draw(0,v.size()-1);
  std::vector<double> means;
  for(int b=0;b<10000;++b){
    double sum=0;for(std::size_t i=0;i<v.size();++i)sum+=v[draw(rng)];
    means.push_back(sum/v.size());
  }
  std::sort(means.begin(),means.end());return {means[249],means[9749]};
}

void write_batch(const std::vector<Episode>& episodes,std::ostream& rows,std::ostream& aggregate,
                 std::ostream& paired,unsigned seed) {
  rows<<"episode_id,method,scenario_id,layout_id,scenario_seed,training_seed,evidence,input_id,declared_terminal,status,reason,start_ns,format,trace_sha256,trace_path,requested,exact,interpolated,outside,gap,max_used_bracket_ms,ate_xy_rmse_m,xy_p95_m,yaw_rmse_deg,rpe_1s_n,rpe_1s_xy_rmse_m,rpe_1s_yaw_rmse_deg,loss_episodes,loss_right_censored,recovery_mean_s\n";
  std::map<Group,std::vector<const Episode*>> groups;
  std::map<std::string,std::map<std::string,const Episode*>> tests;
  for(const auto& e:episodes){
    const auto& p=e.plan;const auto& c=e.coverage;
    groups[{p.evidence,p.scenario,p.method}].push_back(&e);
    if(p.evidence=="test")tests[p.episode][p.method]=&e;
    rows<<p.episode<<','<<p.method<<','<<p.scenario<<','<<p.layout<<','<<p.scenario_seed<<','<<p.training_seed
      <<','<<p.evidence<<','<<p.input_id<<','<<p.terminal<<','<<e.status<<','<<csv_quote(e.reason)<<','
      <<p.start<<','<<p.format<<','<<p.trace_sha256<<','<<csv_quote(p.trace.string())<<','
      <<c.requested<<','<<c.exact<<','<<c.interpolated<<','<<c.outside<<','<<c.gap<<','<<c.max_bracket/1e6;
    if(const auto& s=e.summary){
      rows<<','<<s->ate<<','<<s->p95<<','<<s->yaw_rmse<<','<<s->rpe_n<<',';opt(rows,s->rpe_xy);
      rows<<',';opt(rows,s->rpe_yaw);rows<<','<<s->loss_episodes<<','<<s->loss_censored<<',';opt(rows,s->recovery_mean_s);
    }else rows<<",,,,,,,,,";
    rows<<'\n';
  }
  aggregate<<"evidence,scenario,method,n_planned,n_ok,n_missing,n_crash,n_timeout,n_invalid,n_insufficient,metric,n_metric,mean,median,sample_sd,ci_low,ci_high\n";
  std::mt19937 rng(seed);
  using Metric=std::pair<std::string,std::optional<double>(*)(const Summary&)>;
  const std::vector<Metric> metrics={
    {"ate_xy_rmse_m",[](const Summary& s)->std::optional<double>{return s.ate;}},
    {"xy_p95_m",[](const Summary& s)->std::optional<double>{return s.p95;}},
    {"yaw_rmse_deg",[](const Summary& s)->std::optional<double>{return s.yaw_rmse;}},
    {"rpe_1s_xy_rmse_m",[](const Summary& s){return s.rpe_xy;}},
    {"rpe_1s_yaw_rmse_deg",[](const Summary& s){return s.rpe_yaw;}},
    {"loss_episodes",[](const Summary& s)->std::optional<double>{return s.loss_episodes;}},
  };
  for(const auto& [k,es]:groups){
    const auto& [evidence,scenario,method]=k;
    std::map<std::string,std::size_t> counts;
    std::set<std::string> seen_layouts;
    for(const auto* e:es){++counts[e->status];if(e->summary)seen_layouts.insert(e->plan.layout);}
    for(const auto& [metric,get]:metrics){
      std::vector<double> v;
      for(const auto* e:es)if(e->summary)if(auto x=get(*e->summary)) v.push_back(*x);
      aggregate<<evidence<<','<<scenario<<','<<method<<','<<es.size();
      for(auto key:{"ok","missing","crash","timeout","invalid","insufficient"}) aggregate<<','<<counts[key];
      aggregate<<','<<metric<<','<<v.size();
      if(v.empty())aggregate<<",,,,,";
      else{
        const auto d=describe(v);
        aggregate<<','<<d.mean<<','<<d.median<<',';
        if(v.size()>1)aggregate<<d.sd;
        if(evidence=="test"&&v.size()>=20&&v.size()==es.size()&&seen_layouts.size()>=2){
          const auto ci=bootstrap_mean_ci(v,rng); aggregate<<','<<ci.first<<','<<ci.second;
        }else aggregate<<",,";
      }
      aggregate<<'\n';
    }
  }
  paired<<"record,scenario,episode_id,status,n_pairs,n_layouts,n_planned,delta_B_minus_A_m,median_delta_m,sample_sd_m,ci_low_m,ci_high_m\n";
  std::map<std::string,std::vector<double>> deltas;
  std::map<std::string,std::set<std::string>> layouts,inputs;
  std::map<std::string,std::size_t> planned;
  for(const auto& [id,m]:tests){
    const auto ai=m.find("A"),bi=m.find("B");
    const auto* a=ai==m.end()?nullptr:ai->second;
    const auto* b=bi==m.end()?nullptr:bi->second;
    const auto& scenario=(a?a:b)->plan.scenario;
    ++planned[scenario];
    paired<<"episode,"<<scenario<<','<<id<<',';
    if(!a||!b)paired<<"missing_method,,,,,,,,\n";
    else if(!a->summary||!b->summary)paired<<"failed_method,,,,,,,,\n";
    else if(!compatible_pair(*a,*b))paired<<"metadata_or_truth_mismatch,,,,,,,,\n";
    else if(!inputs[scenario].insert(a->plan.input_id).second)paired<<"duplicate_input_nonindependent,,,,,,,,\n";
    else{
      const double d=b->summary->ate-a->summary->ate;
      layouts[scenario].insert(a->plan.layout);
      deltas[scenario].push_back(d);
      paired<<"paired,,,,"
            <<d<<",,,,\n";
    }
  }
  for(const auto& [scenario,n]:planned){
    const auto& v=deltas[scenario];
    const bool enough=v.size()>=20&&v.size()==n&&layouts[scenario].size()>=2;
    paired<<"summary,"<<scenario<<",,"<<(enough?"ci_available":"incomplete_or_underpowered")
          <<','<<v.size()<<','<<layouts[scenario].size()<<','<<n;
    if(v.empty())paired<<",,,,,";
    else{
      const auto d=describe(v);paired<<','<<d.mean<<','<<d.median<<',';
      if(v.size()>1)paired<<d.sd;
      if(enough){auto ci=bootstrap_mean_ci(v,rng);paired<<','<<ci.first<<','<<ci.second;}
      else paired<<",,";
    }
    paired<<'\n';
  }
}
}  // namespace capstone::g1
