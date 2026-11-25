#include "capstone_benchmark/g1_eval.hpp"

#include <gtest/gtest.h>
#include <cmath>
#include <sstream>
#include <stdexcept>

using namespace capstone::g1;
constexpr Stamp ms=1000000;
constexpr double pi=3.14159265358979323846;
TEST(G1Eval, ParseExplicitFramesAndRejectMalformed) {
  const auto read=[](const char* body,bool legacy=false) {
    std::istringstream in(body);return read_trace(in,legacy);
  };
  constexpr const char* ok="stream,sim_stamp_ns,frame_id,x_m,y_m,yaw_rad\ntruth,1,map,0,0,0\nmap,1,map,0,0,0\nwheel,1,odom,0,0,0\n";
  EXPECT_EQ(read(ok).size(),3u);
  EXPECT_THROW(read("stream,sim_stamp_ns,frame_id,x_m,y_m,yaw_rad\ntruth,1,odom,0,0,0\n"),std::runtime_error);
  EXPECT_THROW(read("stream,sim_stamp_ns,frame_id,x_m,y_m,yaw_rad\ntruth,1,map,nan,0,0\n"),std::runtime_error);
  EXPECT_THROW(read("stream,sim_stamp_ns,frame_id,x_m,y_m,yaw_rad\ntruth,1,map,0,0,0\ntruth,1,map,0,0,0\n"),std::runtime_error);
  EXPECT_THROW(read("stream,sim_stamp_ns,frame_id,x_m,y_m,yaw_rad\ntruth,2,map,0,0,0\ntruth,1,map,0,0,0\n"),std::runtime_error);
  constexpr const char* legacy="stream,sim_stamp_ns,x_m,y_m,yaw_rad\r\ntruth,1,0,0,0\r\nmap,1,0,0,0\r\n";
  EXPECT_THROW(read(legacy),std::runtime_error);
  EXPECT_EQ(read(legacy,true).size(),2u);
}
TEST(G1Eval, OffsetWrapAndRejections) {
  const Stream t={{4*ms,{0,0,179*pi/180}},{24*ms,{2,4,-179*pi/180}},{90*ms,{8,16,0}}};
  const Stream e={{0,{0,0,0}},{4*ms,{0,0,0}},{14*ms,{1,2,pi}},{40*ms,{0,0,0}},{92*ms,{0,0,0}}};
  Coverage c;
  const auto all=pair(t,e,0,c);
  ASSERT_EQ(all.size(),2u);
  EXPECT_EQ(c.requested,5u);EXPECT_EQ(c.exact,1u);EXPECT_EQ(c.interpolated,1u);
  EXPECT_EQ(c.outside,2u);EXPECT_EQ(c.gap,1u);EXPECT_EQ(c.max_bracket,20*ms);
  EXPECT_NEAR(std::abs(all.back().truth.yaw),pi,1e-12);
  EXPECT_DOUBLE_EQ(all.back().truth.x,1.);EXPECT_DOUBLE_EQ(all.back().truth.y,2.);
  EXPECT_TRUE(pair(t,e,100*ms,c).empty());
  EXPECT_TRUE(pair({},e,0,c).empty());EXPECT_EQ(c.outside,5u);
}
TEST(G1Eval, RawATEAndOneSecondRPE) {
  std::vector<Match> a;
  for(Stamp t=0;t<=kSecond;t+=20*ms) {
    const double x=t/static_cast<double>(kSecond);
    a.push_back({t,{x,0,0},{2+1.1*x,0,0}});
  }
  const auto score=summarize(a);
  ASSERT_EQ(score.rpe_n,1u);ASSERT_TRUE(score.rpe_xy);
  EXPECT_NEAR(*score.rpe_xy,.1,1e-12);
  EXPECT_NEAR(score.p95,2.096,1e-12);
  EXPECT_GT(score.ate,2.);
  EXPECT_LT(summarize(align_first(a)).ate,.1);
}
TEST(G1Eval, LossRecoveryAndCensoring) {
  std::vector<Match> a;
  for(Stamp t=0;t<=1200*ms;t+=20*ms) a.push_back({t,{0,0,0},{.6,0,0}});
  const auto ended=summarize(a);
  EXPECT_EQ(ended.loss_episodes,1u);EXPECT_EQ(ended.loss_censored,1u);
  a.push_back({1220*ms,{0,0,0},{0,0,0}});
  const auto recovered=summarize(a);
  EXPECT_EQ(recovered.loss_episodes,1u);EXPECT_EQ(recovered.loss_censored,0u);
  ASSERT_TRUE(recovered.recovery_mean_s);
  EXPECT_DOUBLE_EQ(*recovered.recovery_mean_s,1.22);
}
