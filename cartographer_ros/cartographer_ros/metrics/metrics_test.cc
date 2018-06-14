// TODO

#include "cartographer_ros/metrics/counter.h"
#include "cartographer_ros/metrics/gauge.h"
#include "gtest/gtest.h"

namespace cartographer_ros {
namespace metrics {

TEST(Metrics, testGauge) {
  Gauge gauge;
  EXPECT_EQ(gauge.Value(), 0.);
  gauge.Increment(1.2);
  EXPECT_EQ(gauge.Value(), 1.2);
  gauge.Decrement(2.4);
  EXPECT_EQ(gauge.Value(), -1.2);
}

TEST(Metrics, testCounter) {
  Gauge counter;
  EXPECT_EQ(counter.Value(), 0.);
  gauge.Increment(1.2);
  EXPECT_EQ(counter.Value(), 1.2);
  gauge.Increment(0.8);
  EXPECT_EQ(counter.Value(), 2.0);
}
}
}
