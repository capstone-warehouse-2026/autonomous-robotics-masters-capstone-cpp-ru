#include "capstone_benchmark/g1_batch.hpp"
#include <charconv>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <tuple>

using namespace capstone::g1;
namespace fs=std::filesystem;

int main(int n,char** v) {
  try{
    if(n!=4)throw std::runtime_error("usage: g1_batch_eval MATRIX.csv NEW_OUTPUT_DIR ANALYSIS_SEED");
    unsigned seed{};
    const std::string arg(v[3]);
    const auto p=std::from_chars(arg.data(),arg.data()+arg.size(),seed);
    if(p.ec!=std::errc{}||p.ptr!=arg.data()+arg.size())throw std::runtime_error("invalid analysis seed");
    const auto matrix=fs::canonical(v[1]);
    std::ifstream f(matrix);
    const auto plan=read_plan(f);
    const fs::path out=v[2];
    if(!fs::create_directory(out))throw std::runtime_error("output directory already exists");
    fs::copy_file(matrix,out/"input_matrix.csv");
    std::map<std::tuple<std::string,std::string,std::string>,std::size_t> test_inputs;
    for(const auto& x:plan)if(x.evidence=="test")++test_inputs[{x.method,x.scenario,x.input_id}];
    std::vector<Episode> es;
    for(const auto& x:plan){
      auto e=score_episode(x,matrix.parent_path());
      if(x.evidence=="test"&&test_inputs[{x.method,x.scenario,x.input_id}]>1){
        e.status="invalid";e.reason="same input reused across test episodes within method/scenario";
        e.summary.reset();
      }
      es.push_back(std::move(e));
    }
    std::ofstream rows(out/"episodes.csv"),aggr(out/"aggregates.csv"),paired(out/"paired.csv"),manifest(out/"batch_manifest.json");
    if(!rows||!aggr||!paired||!manifest)throw std::runtime_error("cannot create outputs");
    rows<<std::setprecision(17);aggr<<std::setprecision(17);paired<<std::setprecision(17);
    write_batch(es,rows,aggr,paired,seed);
    manifest<<"{\"schema\":\"g1_offline_batch_v1\",\"analysis_seed\":"<<seed<<",\"bootstrap_resamples\":10000,"
            <<"\"episode_count\":"<<es.size()<<",\"input_hashes\":\"declared in matrix; batch launcher must verify file SHA256\","
            <<"\"evidence_scope\":\"saved outputs; no estimator execution, scene or formal acceptance\"}\n";
    rows.flush();aggr.flush();paired.flush();manifest.flush();
    if(!rows||!aggr||!paired||!manifest)throw std::runtime_error("failed writing results");
    std::size_t ok=0;for(const auto& e:es)if(e.status=="ok")++ok;
    std::cout<<"G1 batch: planned="<<es.size()<<", scored="<<ok<<", unscored="<<es.size()-ok<<'\n';
    return es.size()==ok?0:2;
  }catch(const std::exception& e){
    std::cerr<<"g1_batch_eval: "<<e.what()<<'\n';
    return 1;
  }
}
