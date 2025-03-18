#include <drake/common/autodiff.h>
#include <drake/common/eigen_types.h>
#include <gtest/gtest.h>

#include "dubins.h"

namespace mpcc {
namespace dubins {
namespace test {

using T = double;
using AD = drake::AutoDiffXd;

class DubinsPathTest : public ::testing::Test {
 protected:
  void SetUp() override {
    // Create a simple path with one line segment and one circular segment
    auto ls1 = std::make_shared<LineSegment<T>>(drake::Vector2<T>(10.0, 0.0),
                                                drake::Vector2<T>(10.0, 20.0));

    auto cs1 = std::make_shared<CircularSegment<T>>(
        drake::Vector2<T>(15.0, 20.0), 5.0, 1.0, M_PI, 5.0 * M_PI / 2.0);

    auto ls2 = std::make_shared<LineSegment<T>>(drake::Vector2<T>(15.0, 25.0),
                                                drake::Vector2<T>(35.0, 25.0));

    auto cs2 = std::make_shared<CircularSegment<T>>(
        drake::Vector2<T>(35.0, 35.0), 10.0, -1.0, 0.0, 10.0 * M_PI);

    // Add by copy, of course!
    path_.add_segment(ls1);
    path_.add_segment(cs1);
    path_.add_segment(ls2);
    path_.add_segment(cs2);
  }

  DubinsPath<T> path_;
};

// Test scalar converting constructors
TEST_F(DubinsPathTest, ScalarConversion) {
  // Convert from double to AutoDiffXd
  DubinsPath<AD> ad_path(path_);

  // Check number of segments is preserved
  EXPECT_EQ(ad_path.num_segments(), path_.num_segments());

  // Check first segment is a LineSegment
  auto first_segment = ad_path.get_segment(0);
  EXPECT_NE(std::dynamic_pointer_cast<LineSegment<AD>>(first_segment), nullptr);

  // Check second segment is a CircularSegment
  auto second_segment = ad_path.get_segment(1);
  EXPECT_NE(std::dynamic_pointer_cast<CircularSegment<AD>>(second_segment), nullptr);

  // Check values are preserved
  const auto* line = dynamic_cast<const LineSegment<AD>*>(first_segment.get());
  EXPECT_EQ(line->start().x(), 10.0);
  EXPECT_EQ(line->start().y(), 0.0);
  EXPECT_EQ(line->end().x(), 10.0);
  EXPECT_EQ(line->end().y(), 20.0);

  const auto* circle =
      dynamic_cast<const CircularSegment<AD>*>(second_segment.get());
  EXPECT_EQ(circle->start().x(), 15.0);
  EXPECT_EQ(circle->start().y(), 20.0);
  EXPECT_EQ(circle->length(), 5.0 * M_PI / 2.0);
  EXPECT_EQ(circle->heading_start(), M_PI + M_PI / 2);
  EXPECT_EQ(circle->heading_end(), M_PI + M_PI / 2 + 5.0 * M_PI / 2.0 / 5.0);
}
}  // namespace test
}  // namespace dubins
}  // namespace mpcc

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}