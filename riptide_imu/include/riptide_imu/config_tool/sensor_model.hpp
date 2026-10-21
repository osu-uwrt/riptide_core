#pragma once

#include <Eigen/Geometry>
#include <array>
#include <vector>

namespace riptide_imu {

// The VN-100 Rugged (VN-100x-CR) case, built from the mechanical drawing in the VN-100 user manual (rev 2.45,
// section 2.4) rather than VectorNav's CAD, so the repository carries no third-party model. Units are millimeters
// in the sensor frame: X away from the connector, Y along it, Z down out of the bottom face, origin on the bottom
// face where the manual puts it (15.75 mm from the case's -X face, centered in Y).
enum class ModelMaterial { Case, Connector, Recess, Pin, Print, Hole };

struct ModelTriangle {
  std::array<Eigen::Vector3d, 3> v;  // counterclockwise seen from outside
  ModelMaterial material;
  Eigen::Vector3d normal() const { return (v[1] - v[0]).cross(v[2] - v[0]).normalized(); }
};

const std::vector<ModelTriangle>& vn100RuggedModel();
Eigen::AlignedBox3d vn100RuggedBounds();

}  // namespace riptide_imu
