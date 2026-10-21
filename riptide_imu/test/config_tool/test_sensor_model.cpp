#include <gtest/gtest.h>

#include "riptide_imu/config_tool/sensor_model.hpp"

namespace riptide_imu {

// The model must match the VN-100 user manual's Rugged drawing (section 2.4), millimeters.
TEST(SensorModel, MatchesTheRuggedDrawing) {
  const Eigen::AlignedBox3d bounds = vn100RuggedBounds();
  const Eigen::Vector3d size = bounds.sizes();
  EXPECT_NEAR(size.x(), 33.02, 0.25);  // 27.43 case + 5.59 connector
  EXPECT_NEAR(size.y(), 35.81, 0.25);  // across the mounting tabs (33 mm in the spec is without them)
  EXPECT_NEAR(size.z(), 8.89, 0.25);
  // Origin on the bottom face, 15.75 mm from the case's -X face, centered in Y; Z points down out of the bottom.
  EXPECT_NEAR(bounds.max().z(), 0.0, 1e-9);
  EXPECT_NEAR(bounds.min().x(), -15.75 - 5.59, 1e-9);
  EXPECT_NEAR(bounds.center().y(), 0.0, 1e-9);
}

TEST(SensorModel, ConnectorIsOnMinusXAndPrintOnTop) {
  Eigen::Vector3d connector = Eigen::Vector3d::Zero(), print = Eigen::Vector3d::Zero();
  int connectors = 0, prints = 0;
  for (const auto& triangle : vn100RuggedModel()) {
    const Eigen::Vector3d centroid = (triangle.v[0] + triangle.v[1] + triangle.v[2]) / 3.0;
    if (triangle.material == ModelMaterial::Connector) connector += centroid, ++connectors;
    if (triangle.material == ModelMaterial::Print) {
      print += centroid, ++prints;
      EXPECT_LT(triangle.normal().z(), -0.99) << "printing faces up (-Z)";
    }
  }
  ASSERT_GT(connectors, 0);
  ASSERT_GT(prints, 0);
  EXPECT_LT(connector.x() / connectors, -15.75);
  EXPECT_LT(print.z() / prints, -8.89);
}

// Closed solids wound outward have a positive signed volume (divergence theorem); an inside-out part would
// subtract its volume instead.
TEST(SensorModel, SolidsAreClosedAndWoundOutward) {
  double case_volume = 0.0, connector_volume = 0.0;
  for (const auto& triangle : vn100RuggedModel()) {
    const double volume = triangle.v[0].dot(triangle.v[1].cross(triangle.v[2])) / 6.0;
    if (triangle.material == ModelMaterial::Case) case_volume += volume;
    if (triangle.material == ModelMaterial::Connector) connector_volume += volume;
  }
  // 27.43 x 25.65 x 8.89 mm less the chamfers and top bevel, plus two 2.03 mm tabs (mm^3).
  EXPECT_GT(case_volume, 5400.0);
  EXPECT_LT(case_volume, 6900.0);
  // Housing 5.84 x 20.06 x 6.35 mm plus the latch ridge.
  EXPECT_NEAR(connector_volume, 5.84 * 20.06 * 6.35 + 3.04 * 11.18 * 0.79, 5.0);
}

}  // namespace riptide_imu
