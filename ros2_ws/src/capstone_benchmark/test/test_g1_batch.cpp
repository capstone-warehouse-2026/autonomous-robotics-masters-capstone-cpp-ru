#include "capstone_benchmark/g1_batch.hpp"
#include <gtest/gtest.h>
#include <sstream>
#include <stdexcept>
using namespace capstone::g1;
namespace {
const std::string hdr="episode_id,method,scenario_id,layout_id,scenario_seed,training_seed,evidence,input_id,terminal,start_ns,format,trace_sha256,trace_path\n", hash(64,'a');
std::string missing(std::string id){return id+",A,nominal,warehouse,1,na,smoke,"+hash+",missing,0,framed,-,-\n";}
}
TEST(G1Batch, DuplicateAndMalformedMatrixRejected) {
  std::istringstream ignore(hdr+missing("one")+missing("one"));
  EXPECT_THROW(read_plan(ignore),std::runtime_error);
  std::istringstream bad(hdr+"one,A,nominal,warehouse,-1,na,smoke,"+hash+",missing,0,framed,-,-\n");
  EXPECT_THROW(read_plan(bad),std::runtime_error);
}
TEST(G1Batch, MissingRecordedWithoutZeroMetrics) {
  std::istringstream in(hdr+missing("one"));
  auto p=read_plan(in);
  ASSERT_EQ(p.size(),1u);
  auto e=score_episode(p.front(),".");
  EXPECT_EQ(e.status,"missing");EXPECT_FALSE(e.summary);
  p.front().terminal="ready";p.front().trace="__absent_g1_trace_zz__.csv";
  EXPECT_EQ(score_episode(p.front(),".").status,"missing");
}
TEST(G1Batch, PairMetadataAndTruthMustMatch) {
  Episode a{},b{};
  a.plan.episode=b.plan.episode="test_01"; a.plan.evidence=b.plan.evidence="test";
  a.plan.method="A";b.plan.method="B";
  a.plan.scenario=b.plan.scenario="nominal"; a.plan.layout=b.plan.layout="test_0";
  a.plan.input_id=b.plan.input_id=hash;
  a.truth=b.truth={{0,{0,0,0}}};
  EXPECT_TRUE(compatible_pair(a,b));
  b.truth[0].x=.001;EXPECT_FALSE(compatible_pair(a,b));
  b.truth=a.truth;b.plan.layout="test_1";EXPECT_FALSE(compatible_pair(a,b));
  b.plan.layout=a.plan.layout;b.plan.input_id=std::string(64,'b');EXPECT_FALSE(compatible_pair(a,b));
  b.plan.input_id=hash;b.plan.evidence="fixture";EXPECT_FALSE(compatible_pair(a,b));
}
TEST(G1Batch, DistAndBootstrapAreEpisodeLevel) {
  auto d=describe({1,3}); EXPECT_DOUBLE_EQ(d.mean,2);EXPECT_DOUBLE_EQ(d.median,2);
  EXPECT_NEAR(d.sd,1.41421356237,1e-10);
  std::mt19937 a(20261123),b(20261123);
  EXPECT_EQ(bootstrap_mean_ci({1,3,5},a),bootstrap_mean_ci({1,3,5},b));
  EXPECT_THROW(bootstrap_mean_ci({1},b),std::runtime_error);
}
