#include "riptide_imu/config_tool/sensor_model.hpp"

#include <algorithm>
#include <cmath>

namespace riptide_imu {
namespace {

using V2 = Eigen::Vector2d;
using V3 = Eigen::Vector3d;
using Polygon = std::vector<V2>;

// Dimensions from the manual's VN-100 Rugged drawing, in millimeters. X is measured from the origin.
constexpr double kBodyMinX = -15.75;  // case -X face
constexpr double kBodyMaxX = 11.68;
constexpr double kBodyHalfY = 12.83;
constexpr double kBodyHeight = 8.89;
constexpr double kChamfer = 3.05;  // -X corners narrow to the connector
constexpr double kBevel = 0.76;
constexpr double kConnectorDepth = 5.59;  // Harwin M80 housing beyond the -X face
constexpr double kConnectorHalfY = 10.03;
constexpr double kConnectorHeight = 6.35;
constexpr double kTabThickness = 2.03;
constexpr double kHoleY = 15.42;
constexpr double kTabRadius = 2.49;
constexpr double kScrewX = kBodyMinX + 12.19;
constexpr double kDowelX = kScrewX + 3.81;
constexpr double kPrint = 0.08;  // print layers sit just proud of their face

class Builder {
 public:
  std::vector<ModelTriangle> triangles;

  void tri(V3 a, V3 b, V3 c, const V3& outward, ModelMaterial material) {
    if ((b - a).cross(c - a).dot(outward) < 0) std::swap(b, c);
    if ((b - a).cross(c - a).norm() < 1e-9) return;
    triangles.push_back({{a, b, c}, material});
  }

  void quad(const V3& a, const V3& b, const V3& c, const V3& d, const V3& outward, ModelMaterial material) {
    tri(a, b, c, outward, material);
    tri(a, c, d, outward, material);
  }

  // A solid between two convex outlines with matching vertices (straight walls, or a bevel when they differ).
  void prism(const Polygon& lower, double z_lower, const Polygon& upper, double z_upper, ModelMaterial material,
             bool cap_lower = true, bool cap_upper = true) {
    const V2 center = centroid(lower);
    const std::size_t n = lower.size();
    for (std::size_t i = 0; i < n; ++i) {
      const std::size_t j = (i + 1) % n;
      const V2 mid = 0.5 * (lower[i] + lower[j]);
      const V3 outward(mid.x() - center.x(), mid.y() - center.y(), 0.0);
      quad(at(lower[i], z_lower), at(lower[j], z_lower), at(upper[j], z_upper), at(upper[i], z_upper), outward,
           material);
    }
    const double sign = z_lower > z_upper ? 1.0 : -1.0;
    if (cap_lower) flat(lower, z_lower, sign, material);
    if (cap_upper) flat(upper, z_upper, -sign, material);
  }

  // A convex polygon at height z facing +Z (sign 1, the bottom) or -Z (sign -1, the top).
  void flat(const Polygon& polygon, double z, double sign, ModelMaterial material) {
    const V3 center = at(centroid(polygon), z);
    for (std::size_t i = 0; i < polygon.size(); ++i)
      tri(center, at(polygon[i], z), at(polygon[(i + 1) % polygon.size()], z), V3(0, 0, sign), material);
  }

  void box(const V3& low, const V3& high, ModelMaterial material) {
    const Polygon outline{{low.x(), low.y()}, {high.x(), low.y()}, {high.x(), high.y()}, {low.x(), high.y()}};
    prism(outline, high.z(), outline, low.z(), material);
  }

  // Printed line on the top face (which faces -Z).
  void stroke(const V2& a, const V2& b, double width, double z, ModelMaterial material) {
    const V2 along = (b - a).normalized();
    const V2 side(-along.y() * width / 2, along.x() * width / 2);
    const V2 ea = a - along * width / 2, eb = b + along * width / 2;
    flat({ea + side, eb + side, eb - side, ea - side}, z, -1, material);
  }

  void arrow(const V2& from, const V2& to, double z) {
    const V2 along = (to - from).normalized();
    const V2 side(-along.y(), along.x());
    stroke(from, to - along * 1.02, 0.56, z, ModelMaterial::Print);
    flat({to, to - along * 1.52 + side * 0.89, to - along * 1.52 - side * 0.89}, z, -1, ModelMaterial::Print);
  }

  void disc(const V2& center, double radius, double z, double sign, ModelMaterial material) {
    flat(circle(center, radius, 20), z, sign, material);
  }

  // Rectangle on the connector's front face (which faces -X).
  void front(double x, double y0, double y1, double z0, double z1, ModelMaterial material) {
    quad({x, y0, z0}, {x, y1, z0}, {x, y1, z1}, {x, y0, z1}, V3(-1, 0, 0), material);
  }

  static V3 at(const V2& p, double z) { return {p.x(), p.y(), z}; }

  static V2 centroid(const Polygon& polygon) {
    V2 sum = V2::Zero();
    for (const auto& p : polygon) sum += p;
    return sum / static_cast<double>(polygon.size());
  }

  static Polygon circle(const V2& center, double radius, int pieces) {
    Polygon points;
    for (int i = 0; i < pieces; ++i) {
      const double a = 2 * M_PI * i / pieces;
      points.push_back(center + radius * V2(std::cos(a), std::sin(a)));
    }
    return points;
  }
};

double cross(const V2& o, const V2& a, const V2& b) {
  return (a.x() - o.x()) * (b.y() - o.y()) - (a.y() - o.y()) * (b.x() - o.x());
}

// Counterclockwise convex hull.
Polygon hull(Polygon points) {
  std::sort(points.begin(), points.end(),
            [](const V2& a, const V2& b) { return a.x() < b.x() || (a.x() == b.x() && a.y() < b.y()); });
  Polygon result(2 * points.size());
  std::size_t k = 0;
  for (const auto& p : points) {
    while (k >= 2 && cross(result[k - 2], result[k - 1], p) <= 0) --k;
    result[k++] = p;
  }
  for (std::size_t i = points.size() - 1, t = k + 1; i > 0; --i) {
    while (k >= t && cross(result[k - 2], result[k - 1], points[i - 1]) <= 0) --k;
    result[k++] = points[i - 1];
  }
  result.resize(k - 1);
  return result;
}

// Inward offset of a counterclockwise convex polygon: each vertex moves to where its two edges' offset lines meet.
Polygon inset(const Polygon& polygon, double distance) {
  const std::size_t n = polygon.size();
  Polygon result;
  for (std::size_t i = 0; i < n; ++i) {
    const V2& previous = polygon[(i + n - 1) % n];
    const V2& current = polygon[i];
    const V2& next = polygon[(i + 1) % n];
    const V2 e0 = (current - previous).normalized(), e1 = (next - current).normalized();
    const V2 n0(-e0.y(), e0.x()), n1(-e1.y(), e1.x());  // left of each edge: inward when counterclockwise
    const V2 bisector = (n0 + n1).normalized();
    result.push_back(current + bisector * distance / std::max(0.2, bisector.dot(n0)));
  }
  return result;
}

std::vector<ModelTriangle> build() {
  Builder b;
  const double top = -kBodyHeight;
  const double wall_top = top + kBevel;

  // Case: chamfered toward the connector at -X, with a beveled top edge.
  const Polygon outline = hull({{kBodyMinX + kChamfer, -kBodyHalfY}, {kBodyMaxX, -kBodyHalfY},
                                {kBodyMaxX, kBodyHalfY}, {kBodyMinX + kChamfer, kBodyHalfY},
                                {kBodyMinX, kBodyHalfY - kChamfer}, {kBodyMinX, -kBodyHalfY + kChamfer}});
  const Polygon top_outline = inset(outline, kBevel);
  b.prism(outline, 0.0, outline, wall_top, ModelMaterial::Case, true, false);
  b.prism(outline, wall_top, top_outline, top, ModelMaterial::Case, false, true);

  // Mounting tabs: a 4-40 screw hole and a 1.6 mm dowel hole each, flush with the bottom face.
  for (double side : {-1.0, 1.0}) {
    Polygon points = Builder::circle({kScrewX, side * kHoleY}, kTabRadius, 24);
    for (const auto& p : Builder::circle({kDowelX, side * kHoleY}, kTabRadius, 24)) points.push_back(p);
    points.push_back({kScrewX - kTabRadius, side * (kBodyHalfY - 1.27)});
    points.push_back({kDowelX + kTabRadius, side * (kBodyHalfY - 1.27)});
    b.prism(hull(points), 0.0, hull(points), -kTabThickness, ModelMaterial::Case);
    for (double z_sign : {-1.0, 1.0}) {
      const double z = z_sign < 0 ? -kTabThickness - kPrint : kPrint;
      b.disc({kScrewX, side * kHoleY}, 1.59, z, z_sign, ModelMaterial::Hole);
      b.disc({kDowelX, side * kHoleY}, 0.80, z, z_sign, ModelMaterial::Hole);
    }
  }

  // Harwin M80 10-pin connector on the -X face: housing, latch ridge, mouth and two rows of five pins.
  const double cx = kBodyMinX - kConnectorDepth;
  b.box({cx, -kConnectorHalfY, -0.76 - kConnectorHeight}, {kBodyMinX + 0.25, kConnectorHalfY, -0.76},
        ModelMaterial::Connector);
  b.box({cx + 1.02, -5.59, -7.87}, {cx + 4.06, 5.59, -0.76 - kConnectorHeight + 0.03}, ModelMaterial::Connector);
  b.front(cx - kPrint, -8.38, 8.38, -1.78, -6.10, ModelMaterial::Recess);
  for (int row = 0; row < 2; ++row) {
    for (int column = 0; column < 5; ++column) {
      const double y = -4.01 + column * 2.01, z = row ? -4.83 : -3.05;
      b.front(cx - 2 * kPrint, y - 0.41, y + 0.41, z - 0.41, z + 0.41, ModelMaterial::Pin);
    }
  }

  // Top print: the VectorNav mark (triangle and diamond) and the axis legend, as on the case.
  const double z = top - kPrint;
  const V2 a(-2.03, -4.83), c(-2.03, 4.83), apex(5.08, 0.0);
  b.stroke(a, c, 0.89, z, ModelMaterial::Print);
  b.stroke(c, apex, 0.89, z, ModelMaterial::Print);
  b.stroke(apex, a, 0.89, z, ModelMaterial::Print);
  b.flat({{5.08, 0.0}, {7.62, -1.91}, {10.16, 0.0}, {7.62, 1.91}}, z, -1, ModelMaterial::Print);
  const V2 g(-10.16, 0.0);
  b.arrow(g, g + V2(4.32, 0.0), z);
  b.arrow(g, g + V2(0.0, 4.32), z);
  for (int i = 0; i < 16; ++i) {
    const double a0 = 2 * M_PI * i / 16, a1 = 2 * M_PI * (i + 1) / 16;
    b.stroke(g + 1.14 * V2(std::cos(a0), std::sin(a0)), g + 1.14 * V2(std::cos(a1), std::sin(a1)), 0.41, z,
             ModelMaterial::Print);
  }
  b.stroke(g + V2(-0.76, -0.76), g + V2(0.76, 0.76), 0.36, z, ModelMaterial::Print);
  b.stroke(g + V2(-0.76, 0.76), g + V2(0.76, -0.76), 0.36, z, ModelMaterial::Print);
  const V2 x_label = g + V2(6.10, 0.0), y_label = g + V2(0.0, 6.10);
  b.stroke(x_label + V2(-0.76, -0.89), x_label + V2(0.76, 0.89), 0.41, z, ModelMaterial::Print);
  b.stroke(x_label + V2(-0.76, 0.89), x_label + V2(0.76, -0.89), 0.41, z, ModelMaterial::Print);
  b.stroke(y_label + V2(-0.76, -0.89), y_label, 0.41, z, ModelMaterial::Print);
  b.stroke(y_label + V2(0.76, -0.89), y_label, 0.41, z, ModelMaterial::Print);
  b.stroke(y_label, y_label + V2(0.0, 1.02), 0.41, z, ModelMaterial::Print);
  return b.triangles;
}

}  // namespace

const std::vector<ModelTriangle>& vn100RuggedModel() {
  static const std::vector<ModelTriangle> model = build();
  return model;
}

Eigen::AlignedBox3d vn100RuggedBounds() {
  Eigen::AlignedBox3d bounds;
  for (const auto& triangle : vn100RuggedModel()) {
    if (triangle.material != ModelMaterial::Case && triangle.material != ModelMaterial::Connector) continue;
    for (const auto& vertex : triangle.v) bounds.extend(vertex);
  }
  return bounds;
}

}  // namespace riptide_imu
