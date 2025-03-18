#include <gtest/gtest.h>
#include <drake/common/autodiff.h>
#include <drake/common/eigen_types.h>
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
    auto line = std::make_unique<LineSegment<T>>(
        drake::Vector2<T>(0.0, 0.0),
        drake::Vector2<T>(1.0, 1.0));
    path_.add_segment(std::move(line));

    auto circle = std::make_unique<CircularSegment<T>>(
        drake::Vector2<T>(1.0, 1.0),  // center
        1.0,  // radius
        1.0,  // dir
        0.0,  // heading
        M_PI / 2.0);  // arclength
    path_.add_segment(std::move(circle));
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
  const auto& first_segment = ad_path.get_segment(0);
  EXPECT_NE(dynamic_cast<const LineSegment<AD>*>(&first_segment), nullptr);
  
  // Check second segment is a CircularSegment
  const auto& second_segment = ad_path.get_segment(1);
  EXPECT_NE(dynamic_cast<const CircularSegment<AD>*>(&second_segment), nullptr);
  
  // Check values are preserved
  const auto* line = dynamic_cast<const LineSegment<AD>*>(&first_segment);
  EXPECT_EQ(line->start.x(), 0.0);
  EXPECT_EQ(line->start.y(), 0.0);
  EXPECT_EQ(line->end.x(), 1.0);
  EXPECT_EQ(line->end.y(), 1.0);
  
  const auto* circle = dynamic_cast<const CircularSegment<AD>*>(&second_segment);
  EXPECT_EQ(circle->center.x(), 1.0);
  EXPECT_EQ(circle->center.y(), 1.0);
  EXPECT_EQ(circle->radius, 1.0);
  EXPECT_EQ(circle->dir, CircularDirection::NorthEast);
  EXPECT_EQ(circle->heading, 0.0);
  EXPECT_EQ(circle->arclength, M_PI / 2.0);
}

// Test path_coords for LineSegment
TEST_F(DubinsPathTest, LineSegmentPathCoords) {
  const auto& line = dynamic_cast<const LineSegment<T>&>(path_.get_segment(0));
  
  // Test point at start
  auto coords = line.path_coords(drake::Vector2<T>(0.0, 0.0));
  EXPECT_EQ(coords.x(), 0.0);
  EXPECT_EQ(coords.y(), 0.0);
  
  // Test point at end
  coords = line.path_coords(drake::Vector2<T>(1.0, 1.0));
  EXPECT_EQ(coords.x(), std::sqrt(2.0));
  EXPECT_EQ(coords.y(), 0.0);
  
  // Test point halfway
  coords = line.path_coords(drake::Vector2<T>(0.5, 0.5));
  EXPECT_EQ(coords.x(), std::sqrt(2.0) / 2.0);
  EXPECT_EQ(coords.y(), 0.0);
  
  // Test point off the line
  coords = line.path_coords(drake::Vector2<T>(0.5, 1.0));
  EXPECT_EQ(coords.x(), std::sqrt(2.0) / 2.0);
  EXPECT_EQ(coords.y(), 0.5);
}

// Test path_coords for CircularSegment
TEST_F(DubinsPathTest, CircularSegmentPathCoords) {
  const auto& circle = dynamic_cast<const CircularSegment<T>&>(path_.get_segment(1));
  
  // Test point at center
  auto coords = circle.path_coords(drake::Vector2<T>(1.0, 1.0));
  EXPECT_EQ(coords.x(), 0.0);
  EXPECT_EQ(coords.y(), 0.0);
  
  // Test point on circle at start
  coords = circle.path_coords(drake::Vector2<T>(2.0, 1.0));
  EXPECT_EQ(coords.x(), 0.0);
  EXPECT_EQ(coords.y(), 0.0);
  
  // Test point on circle at end
  coords = circle.path_coords(drake::Vector2<T>(1.0, 2.0));
  EXPECT_EQ(coords.x(), M_PI / 2.0);
  EXPECT_EQ(coords.y(), 0.0);
  
  // Test point off circle
  coords = circle.path_coords(drake::Vector2<T>(2.0, 2.0));
  EXPECT_EQ(coords.x(), M_PI / 4.0);
  EXPECT_EQ(coords.y(), std::sqrt(2.0) - 1.0);
}

}  // namespace test
}  // namespace dubins
}  // namespace mpcc 