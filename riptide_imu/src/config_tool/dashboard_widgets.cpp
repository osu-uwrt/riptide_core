#include "riptide_imu/config_tool/dashboard_widgets.hpp"

#include <QLabel>
#include <QMouseEvent>
#include <QPainter>
#include <QPainterPath>
#include <QWheelEvent>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <vector>

#include "riptide_imu/config_tool/sensor_model.hpp"
#include "riptide_imu/config_tool/theme.hpp"

namespace riptide_imu {
namespace {

constexpr double kDefaultAzimuth = -130.0;
constexpr double kDefaultElevation = 24.0;
constexpr double kDefaultDistance = 5.4;
constexpr double kNear = 0.05;
// Scene units per millimeter: the 33 mm VN-100 Rugged (with connector) is about two units long.
constexpr double kModelScale = 0.0571;

double radians(double degrees) { return degrees * M_PI / 180.0; }

// Perspective projection from the camera's position into a widget rectangle.
class Projector {
 public:
  Projector(const QRectF& rect, const SceneCamera& camera, const Eigen::Vector3d& target) : rect_(rect) {
    const double az = radians(camera.azimuth_deg);
    const double el = radians(std::clamp(camera.elevation_deg, -10.0, 88.0));
    eye_ = target + camera.distance *
                        Eigen::Vector3d(std::cos(el) * std::cos(az), std::cos(el) * std::sin(az), std::sin(el));
    forward_ = (target - eye_).normalized();
    right_ = forward_.cross(Eigen::Vector3d::UnitZ()).normalized();
    up_ = right_.cross(forward_);
    focal_ = std::min(rect.width(), rect.height()) * 1.15;
  }

  Eigen::Vector3d camera(const Eigen::Vector3d& world) const {
    const Eigen::Vector3d v = world - eye_;
    return {v.dot(right_), v.dot(up_), v.dot(forward_)};
  }
  QPointF screen(const Eigen::Vector3d& c) const {
    return {rect_.center().x() + focal_ * c.x() / c.z(), rect_.center().y() - focal_ * c.y() / c.z()};
  }
  bool project(const Eigen::Vector3d& world, QPointF* out) const {
    const Eigen::Vector3d c = camera(world);
    if (c.z() < kNear) return false;
    *out = screen(c);
    return true;
  }
  double depth(const Eigen::Vector3d& world) const { return camera(world).z(); }
  // Clips a world segment against the near plane; false when it is wholly behind the camera.
  bool segment(const Eigen::Vector3d& a, const Eigen::Vector3d& b, QPointF* pa, QPointF* pb) const {
    Eigen::Vector3d ca = camera(a), cb = camera(b);
    if (ca.z() < kNear && cb.z() < kNear) return false;
    if (ca.z() < kNear) ca = ca + (cb - ca) * ((kNear - ca.z()) / (cb.z() - ca.z()));
    if (cb.z() < kNear) cb = cb + (ca - cb) * ((kNear - cb.z()) / (ca.z() - cb.z()));
    *pa = screen(ca);
    *pb = screen(cb);
    return true;
  }
  const Eigen::Vector3d& eye() const { return eye_; }
  const Eigen::Vector3d& forward() const { return forward_; }
  const Eigen::Vector3d& up() const { return up_; }
  const Eigen::Vector3d& right() const { return right_; }

 private:
  QRectF rect_;
  Eigen::Vector3d eye_, forward_, right_, up_;
  double focal_;
};

double cross2(const QPointF& o, const QPointF& a, const QPointF& b) {
  return (a.x() - o.x()) * (b.y() - o.y()) - (a.y() - o.y()) * (b.x() - o.x());
}

QPolygonF convexHull(std::vector<QPointF> points) {
  std::sort(points.begin(), points.end(), [](const QPointF& a, const QPointF& b) {
    return a.x() < b.x() || (a.x() == b.x() && a.y() < b.y());
  });
  if (points.size() < 3) return QPolygonF(QVector<QPointF>(points.begin(), points.end()));
  std::vector<QPointF> hull(2 * points.size());
  std::size_t k = 0;
  for (const auto& p : points) {
    while (k >= 2 && cross2(hull[k - 2], hull[k - 1], p) <= 0) --k;
    hull[k++] = p;
  }
  for (std::size_t i = points.size() - 1, t = k + 1; i > 0; --i) {
    while (k >= t && cross2(hull[k - 2], hull[k - 1], points[i - 1]) <= 0) --k;
    hull[k++] = points[i - 1];
  }
  hull.resize(k - 1);
  return QPolygonF(QVector<QPointF>(hull.begin(), hull.end()));
}

void arrowHead(QPainter& painter, const QPointF& tail, const QPointF& tip, const QColor& color, double size) {
  QPointF d = tip - tail;
  const double length = std::hypot(d.x(), d.y());
  if (length < 1e-3) return;
  d /= length;
  const QPointF normal(-d.y(), d.x());
  QPolygonF head;
  head << tip << tip - d * size + normal * size * 0.5 << tip - d * size - normal * size * 0.5;
  painter.setPen(Qt::NoPen);
  painter.setBrush(color);
  painter.drawPolygon(head);
}

// Software z-buffer for the sensor model and axes: supersampled, unlit, with edges where the normal or depth changes.
class Raster {
 public:
  Raster(const Projector& projector, const QRectF& rect, double supersample)
      : projector_(projector), origin_(rect.topLeft()), scale_(supersample),
        width_(std::max(1, static_cast<int>(std::ceil(rect.width() * supersample)))),
        height_(std::max(1, static_cast<int>(std::ceil(rect.height() * supersample)))),
        color_(width_, height_, QImage::Format_ARGB32_Premultiplied),
        depth_(static_cast<std::size_t>(width_) * height_, 0.0f),
        key_(static_cast<std::size_t>(width_) * height_, 0u) {
    color_.fill(Qt::transparent);
  }

  // culled when facing away
  void triangle(const Eigen::Vector3d& a, const Eigen::Vector3d& b, const Eigen::Vector3d& c, QRgb color,
                const Eigen::Vector3d& normal) {
    if (normal.dot(projector_.eye() - a) <= 0) return;
    const Eigen::Vector3d ca = projector_.camera(a), cb = projector_.camera(b), cc = projector_.camera(c);
    if (ca.z() < kNear || cb.z() < kNear || cc.z() < kNear) return;
    const QPointF pa = pixel(projector_.screen(ca)), pb = pixel(projector_.screen(cb)),
                  pc = pixel(projector_.screen(cc));
    const double area = edge(pa, pb, pc);
    if (std::abs(area) < 1e-9) return;
    // plane (normal and distance) as a key; neighbors with different keys are different faces
    const auto q = [](double v) { return static_cast<std::uint32_t>(std::lround((v + 1.0) * 40.0)); };
    const auto offset = static_cast<std::uint32_t>(std::lround(normal.dot(a) / 0.03) + 200);
    const std::uint32_t key =
        1u + q(normal.x()) + 81u * q(normal.y()) + 6561u * q(normal.z()) + 531441u * offset;
    const int x0 = std::max(0, static_cast<int>(std::floor(std::min({pa.x(), pb.x(), pc.x()}))));
    const int x1 = std::min(width_ - 1, static_cast<int>(std::ceil(std::max({pa.x(), pb.x(), pc.x()}))));
    const int y0 = std::max(0, static_cast<int>(std::floor(std::min({pa.y(), pb.y(), pc.y()}))));
    const int y1 = std::min(height_ - 1, static_cast<int>(std::ceil(std::max({pa.y(), pb.y(), pc.y()}))));
    const double iza = 1.0 / ca.z(), izb = 1.0 / cb.z(), izc = 1.0 / cc.z();
    for (int y = y0; y <= y1; ++y) {
      auto* row = reinterpret_cast<QRgb*>(color_.scanLine(y));
      for (int x = x0; x <= x1; ++x) {
        const QPointF p(x + 0.5, y + 0.5);
        const double w0 = edge(pb, pc, p) / area, w1 = edge(pc, pa, p) / area, w2 = 1.0 - w0 - w1;
        if (w0 < -1e-7 || w1 < -1e-7 || w2 < -1e-7) continue;
        const float inverse_depth = static_cast<float>(w0 * iza + w1 * izb + w2 * izc);
        const std::size_t index = static_cast<std::size_t>(y) * width_ + x;
        if (inverse_depth <= depth_[index]) continue;
        depth_[index] = inverse_depth;
        key_[index] = key;
        row[x] = color;
      }
    }
  }

  QImage resolve(const QColor& edge_color, double device_ratio) {
    const QRgb ink = edge_color.rgba();
    std::vector<std::uint8_t> boundary(depth_.size(), 0);
    for (int y = 0; y < height_; ++y) {
      for (int x = 0; x < width_; ++x) {
        const std::size_t index = static_cast<std::size_t>(y) * width_ + x;
        if (!key_[index]) continue;
        for (const auto& [dx, dy] : {std::pair{1, 0}, std::pair{0, 1}, std::pair{-1, 0}, std::pair{0, -1}}) {
          const int nx = x + dx, ny = y + dy;
          if (nx < 0 || ny < 0 || nx >= width_ || ny >= height_) continue;
          const std::size_t other = static_cast<std::size_t>(ny) * width_ + nx;
          // no depth test, or edge-on faces count as all edge
          if (!key_[other] || (key_[other] != key_[index] && key_[other] < key_[index])) {
            boundary[index] = 1;
            break;
          }
        }
      }
    }
    for (int y = 0; y < height_; ++y) {
      auto* row = reinterpret_cast<QRgb*>(color_.scanLine(y));
      for (int x = 0; x < width_; ++x) {
        if (!boundary[static_cast<std::size_t>(y) * width_ + x]) continue;
        const QRgb c = row[x];
        row[x] = qRgba((qRed(c) + 2 * qRed(ink)) / 3, (qGreen(c) + 2 * qGreen(ink)) / 3,
                       (qBlue(c) + 2 * qBlue(ink)) / 3, 255);
      }
    }
    QImage image = color_.scaled(static_cast<int>(std::round(width_ / scale_ * device_ratio)),
                                 static_cast<int>(std::round(height_ / scale_ * device_ratio)),
                                 Qt::IgnoreAspectRatio, Qt::SmoothTransformation);
    image.setDevicePixelRatio(device_ratio);
    return image;
  }

 private:
  QPointF pixel(const QPointF& widget) const { return (widget - origin_) * scale_; }
  static double edge(const QPointF& a, const QPointF& b, const QPointF& p) {
    return (b.x() - a.x()) * (p.y() - a.y()) - (b.y() - a.y()) * (p.x() - a.x());
  }

  const Projector& projector_;
  QPointF origin_;
  double scale_;
  int width_, height_;
  QImage color_;
  std::vector<float> depth_;
  std::vector<std::uint32_t> key_;
};

QColor materialColor(ModelMaterial material) {
  switch (material) {
    case ModelMaterial::Case: return QColor("#a3202a");  // VectorNav red anodize
    case ModelMaterial::Connector: return QColor("#2b2d31");
    case ModelMaterial::Recess: return QColor("#111214");
    case ModelMaterial::Pin: return QColor("#c9a43a");
    case ModelMaterial::Print: return QColor("#f2f2f2");
    case ModelMaterial::Hole: return QColor("#0c0c0d");
  }
  return Qt::gray;
}


}  // namespace

QLabel* sectionTitle(const QString& text) {
  auto* label = new QLabel(text);
  label->setObjectName("sectionTitle");
  return label;
}

QLabel* toneLabel(const QString& text, const QString& tone) {
  auto* label = new QLabel(text);
  label->setProperty("tone", tone);
  label->setWordWrap(true);
  return label;
}

StatusChip::StatusChip(const QString& text, Level level, bool on_board, QWidget* parent)
    : QWidget(parent), text_(text), level_(level), on_board_(on_board) {
  setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
}

void StatusChip::setState(const QString& text, Level level) {
  if (text == text_ && level == level_) return;
  text_ = text;
  level_ = level;
  updateGeometry();
  update();
}

void StatusChip::reserveWidthFor(const QStringList& texts) {
  reserved_width_ = 0;
  for (const auto& text : texts) reserved_width_ = std::max(reserved_width_, static_cast<int>(statusChipWidth(text)));
  updateGeometry();
}

QSize StatusChip::sizeHint() const {
  return {std::max(reserved_width_, static_cast<int>(statusChipWidth(text_))),
          QFontMetrics(strongFont(12)).height() + 8};
}

void StatusChip::paintEvent(QPaintEvent*) {
  QPainter painter(this);
  paintStatusChip(painter, rect(), text_, level_, on_board_);
}

double statusChipWidth(const QString& text) {
  return QFontMetrics(strongFont(12)).horizontalAdvance(text.toUpper()) + 18;
}

void paintStatusChip(QPainter& painter, const QRectF& rect, const QString& text, StatusChip::Level level,
                     bool on_board) {
  using Level = StatusChip::Level;
  const auto& t = theme();
  QColor color;
  switch (level) {
    case Level::Neutral: color = on_board ? t.board_muted : t.muted; break;
    case Level::Info: color = on_board ? t.board_accent : t.accent; break;
    case Level::Ok: color = on_board ? t.board_ok : t.ok; break;
    case Level::Warn: color = on_board ? t.board_warn : t.warn; break;
    case Level::Error: color = t.error; break;
    case Level::Change: color = t.change; break;
  }
  painter.save();
  painter.setRenderHint(QPainter::Antialiasing);
  const QRectF box = rect.adjusted(0.5, 0.5, -0.5, -0.5);
  if (level == Level::Change) {
    painter.setPen(Qt::NoPen);
    painter.setBrush(t.change);
    painter.drawRoundedRect(box, 2, 2);
    painter.setPen(t.change_text);
  } else {
    QColor fill = color;
    fill.setAlphaF(0.12);
    painter.setPen(QPen(color, 1.0));
    painter.setBrush(fill);
    painter.drawRoundedRect(box, 2, 2);
    painter.setPen(color);
  }
  painter.setFont(strongFont(12));
  painter.drawText(rect, Qt::AlignCenter, text.toUpper());
  painter.restore();
}

SegmentedSwitch::SegmentedSwitch(const QStringList& options, bool on_board, QWidget* parent)
    : QWidget(parent), options_(options), on_board_(on_board) {
  setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
  setCursor(Qt::PointingHandCursor);
  setFocusPolicy(Qt::TabFocus);
}

void SegmentedSwitch::setCurrentIndex(int index) {
  if (index < 0 || index >= options_.size() || index == index_) return;
  index_ = index;
  update();
  emit currentIndexChanged(index_);
}

QSize SegmentedSwitch::sizeHint() const {
  const QFontMetrics metrics(uiFont(14));
  int width = 4;
  for (const auto& option : options_) width += metrics.horizontalAdvance(option) + 24;
  return {width, metrics.height() + 14};
}

QRectF SegmentedSwitch::segment(int index) const {
  const QFontMetrics metrics(uiFont(14));
  double x = 2;
  for (int i = 0; i < index; ++i) x += metrics.horizontalAdvance(options_[i]) + 24;
  return {x, 2, static_cast<double>(metrics.horizontalAdvance(options_[index]) + 24), height() - 4.0};
}

void SegmentedSwitch::paintEvent(QPaintEvent*) {
  const auto& t = theme();
  QPainter painter(this);
  painter.setRenderHint(QPainter::Antialiasing);
  const QRectF box = QRectF(rect()).adjusted(0.5, 0.5, -0.5, -0.5);
  painter.setPen(QPen(on_board_ ? t.board_border : t.border, 1));
  painter.setBrush(on_board_ ? t.board_frame : t.frame);
  painter.drawRoundedRect(box, 2, 2);
  painter.setPen(Qt::NoPen);
  QColor thumb = t.active;
  if (!isEnabled()) thumb = blend(on_board_ ? t.board_frame : t.frame, t.active, 0.55);
  painter.setBrush(thumb);
  painter.drawRoundedRect(segment(index_), 1.5, 1.5);
  painter.setFont(uiFont(14));
  for (int i = 0; i < options_.size(); ++i) {
    painter.setPen(i == index_ ? t.active_text : (on_board_ ? t.board_text : t.text));
    if (!isEnabled() && i != index_) painter.setPen(on_board_ ? t.board_muted : t.muted);
    painter.drawText(segment(i), Qt::AlignCenter, options_[i]);
  }
}

void SegmentedSwitch::mousePressEvent(QMouseEvent* event) {
  for (int i = 0; i < options_.size(); ++i) {
    if (!segment(i).contains(event->pos())) continue;
    setCurrentIndex(i);
    emit activated(i);
  }
}

Eigen::Matrix3d nedToEnu() {
  Eigen::Matrix3d m;
  m << 0, 1, 0, 1, 0, 0, 0, 0, -1;
  return m;
}

Eigen::Vector3d eulerDeg(const Eigen::Matrix3d& r) {
  const double roll = std::atan2(r(2, 1), r(2, 2));
  const double pitch = -std::asin(std::clamp(r(2, 0), -1.0, 1.0));
  const double yaw = std::atan2(r(1, 0), r(0, 0));
  return Eigen::Vector3d(roll, pitch, yaw) * 180.0 / M_PI;
}

void paintSensorScene(QPainter& painter, const QRectF& rect, const Eigen::Matrix3d& body_to_world,
                      const SceneCamera& camera, const SceneOptions& options,
                      const std::optional<Eigen::Matrix3d>& ghost) {
  const auto& t = theme();
  const double gz = options.ground_z;
  const auto& model = vn100RuggedModel();
  const Eigen::Vector3d model_center = body_to_world * (vn100RuggedBounds().center() * kModelScale);
  const Projector projector(rect, camera, model_center);
  painter.save();
  painter.setClipRect(rect);
  painter.setRenderHint(QPainter::Antialiasing);
  const double origin_depth = projector.depth(Eigen::Vector3d::Zero());

  // ground grid
  if (options.grid) {
    for (int i = -8; i <= 8; ++i) {
      const double c = i * 0.45;
      for (int direction = 0; direction < 2; ++direction) {
        const Eigen::Vector3d a = direction ? Eigen::Vector3d(c, -3.6, gz) : Eigen::Vector3d(-3.6, c, gz);
        const Eigen::Vector3d b = direction ? Eigen::Vector3d(c, 3.6, gz) : Eigen::Vector3d(3.6, c, gz);
        QPointF pa, pb;
        if (!projector.segment(a, b, &pa, &pb)) continue;
        QColor color = i == 0 ? t.muted : t.border;
        color.setAlphaF(i == 0 ? 0.55 : std::max(0.15, 0.9 - std::abs(i) * 0.09));
        painter.setPen(QPen(color, i == 0 ? 1.4 : 1.0));
        painter.drawLine(pa, pb);
      }
    }
    if (options.compass) {
      const std::array<std::pair<const char*, Eigen::Vector3d>, 4> letters{
          {{"N", {0, 4.0, gz}}, {"E", {4.0, 0, gz}}, {"S", {0, -4.0, gz}}, {"W", {-4.0, 0, gz}}}};
      painter.setFont(strongFont(14));
      for (const auto& [letter, position] : letters) {
        QPointF point;
        if (!projector.project(position, &point)) continue;
        painter.setPen(letter[0] == 'N' ? t.text : t.muted);
        painter.drawText(QRectF(point.x() - 12, point.y() - 10, 24, 20), Qt::AlignCenter, letter);
      }
    }
  }

  if (options.shadow) {
    std::vector<QPointF> points;
    for (const auto& triangle : model) {
      if (triangle.material != ModelMaterial::Case && triangle.material != ModelMaterial::Connector) continue;
      for (const auto& vertex : triangle.v) {
        const Eigen::Vector3d w = body_to_world * (vertex * kModelScale);
        QPointF point;
        if (projector.project({w.x(), w.y(), gz}, &point)) points.push_back(point);
      }
    }
    if (points.size() >= 3) {
      painter.setPen(Qt::NoPen);
      painter.setBrush(QColor(0, 0, 0, t.dark ? 70 : 30));
      painter.drawPolygon(convexHull(points));
    }
  }

  // level ring, split so the near half draws over the case
  std::vector<std::pair<QLineF, bool>> ring;
  if (options.horizon) {
    const int pieces = 96;
    for (int i = 0; i < pieces; ++i) {
      if (i % 2) continue;
      const double a0 = 2 * M_PI * i / pieces, a1 = 2 * M_PI * (i + 1) / pieces;
      const Eigen::Vector3d p0(1.9 * std::cos(a0), 1.9 * std::sin(a0), 0);
      const Eigen::Vector3d p1(1.9 * std::cos(a1), 1.9 * std::sin(a1), 0);
      QPointF s0, s1;
      if (!projector.segment(p0, p1, &s0, &s1)) continue;
      ring.push_back({QLineF(s0, s1), projector.depth(0.5 * (p0 + p1)) < origin_depth});
    }
  }
  auto drawRing = [&](bool front) {
    QColor color = t.accent;
    color.setAlphaF(0.55);
    painter.setPen(QPen(color, 1.6));
    for (const auto& [line, is_front] : ring)
      if (is_front == front) painter.drawLine(line);
  };
  drawRing(false);

  if (options.forward) {
    // robot front
    const Eigen::Vector3d ahead = Eigen::Vector3d(options.forward->x(), options.forward->y(), 0).normalized();
    const Eigen::Vector3d side(-ahead.y(), ahead.x(), 0);
    const double z = gz + 0.02;
    QPointF tail, tip, left, right, label;
    if (ahead.allFinite() && projector.project(ahead * 1.2 + Eigen::Vector3d(0, 0, z), &tail) &&
        projector.project(ahead * 2.9 + Eigen::Vector3d(0, 0, z), &tip) &&
        projector.project(ahead * 2.45 + side * 0.32 + Eigen::Vector3d(0, 0, z), &left) &&
        projector.project(ahead * 2.45 - side * 0.32 + Eigen::Vector3d(0, 0, z), &right) &&
        projector.project(ahead * 3.35 + Eigen::Vector3d(0, 0, z), &label)) {
      QColor color = t.text;
      color.setAlphaF(0.75);
      painter.setPen(QPen(color, 3.0, Qt::SolidLine, Qt::RoundCap));
      painter.drawLine(tail, (left + right) / 2);
      painter.setPen(Qt::NoPen);
      painter.setBrush(color);
      painter.drawPolygon(QPolygonF({tip, left, right}));
      painter.setFont(strongFont(12));
      painter.setPen(t.text);
      const double w = QFontMetrics(strongFont(12)).horizontalAdvance(options.forward_label) + 8;
      painter.drawText(QRectF(label.x() - w / 2, label.y() - 9, w, 18), Qt::AlignCenter, options.forward_label);
    }
  }

  if (options.turn_arrow) {
    QColor color = t.accent;
    QPainterPath path;
    QPointF last, before_last;
    bool started = false;
    const double z = gz + 0.02;
    for (int i = 0; i <= 54; ++i) {
      const double a = M_PI * 0.25 + i * (1.5 * M_PI / 54);
      QPointF point;
      if (!projector.project({2.3 * std::cos(a), 2.3 * std::sin(a), z}, &point)) continue;
      started ? path.lineTo(point) : path.moveTo(point);
      started = true;
      before_last = last;
      last = point;
    }
    painter.setPen(QPen(color, 4.0, Qt::SolidLine, Qt::RoundCap));
    painter.setBrush(Qt::NoBrush);
    painter.drawPath(path);
    arrowHead(painter, before_last, last + (last - before_last) * 2.0, color, 18);
  }

  // case and axes share one depth buffer; unlit, faces told apart by edge lines
  const double device_ratio = painter.device() ? painter.device()->devicePixelRatioF() : 1.0;
  auto drawModel = [&](const Eigen::Matrix3d& rotation, const std::optional<QColor>& tint, bool with_axes) {
    Raster raster(projector, rect, 2.0 * device_ratio);
    for (const auto& triangle : model) {
      if (!options.decal && triangle.material == ModelMaterial::Print) continue;
      const Eigen::Vector3d a = rotation * (triangle.v[0] * kModelScale);
      const Eigen::Vector3d b = rotation * (triangle.v[1] * kModelScale);
      const Eigen::Vector3d c = rotation * (triangle.v[2] * kModelScale);
      const Eigen::Vector3d normal = rotation * triangle.normal();
      raster.triangle(a, b, c, (tint ? *tint : materialColor(triangle.material)).rgba(), normal);
    }
    if (with_axes) {
      for (int axis = 0; axis < 3; ++axis) {
        const Eigen::Vector3d d = rotation * Eigen::Vector3d::Unit(axis);
        const Eigen::Vector3d u = d.unitOrthogonal(), v = d.cross(u);
        const QRgb color = axisColor(axis).rgba();
        const double shaft = 0.028, head = 0.085, neck = 1.55, tip = 1.85;
        const Eigen::Vector3d corners[4] = {u + v, -u + v, -u - v, u - v};
        for (int i = 0; i < 4; ++i) {
          const Eigen::Vector3d p = corners[i] * shaft, q = corners[(i + 1) % 4] * shaft;
          const Eigen::Vector3d n = (p + q).normalized();
          raster.triangle(p, q, q + d * neck, color, n);
          raster.triangle(p, q + d * neck, p + d * neck, color, n);
          const Eigen::Vector3d hp = d * neck + corners[i] * head, hq = d * neck + corners[(i + 1) % 4] * head;
          const Eigen::Vector3d hn = (hq - hp).cross(d * tip - hp).normalized();
          const Eigen::Vector3d facing = hn.dot((hp + hq) / 2 - d * neck) < 0 ? Eigen::Vector3d(-hn) : hn;
          raster.triangle(hp, hq, d * tip, color, facing);
          raster.triangle(hq, hp, d * neck, color, -d);
        }
      }
    }
    return raster.resolve(tint ? tint->darker(170) : QColor("#141518"), device_ratio);
  };
  painter.drawImage(rect.topLeft(), drawModel(body_to_world, std::nullopt, options.axes));
  if (ghost) {
    // sensor now, translucent
    painter.setOpacity(0.38);
    painter.drawImage(rect.topLeft(), drawModel(*ghost, t.accent, false));
    painter.setOpacity(1.0);
  }

  drawRing(true);
  if (options.axes) {
    painter.setFont(strongFont(12));
    for (int axis = 0; axis < 3; ++axis) {
      QPointF label;
      if (!projector.project(body_to_world * Eigen::Vector3d::Unit(axis) * 2.08, &label)) continue;
      painter.setPen(Qt::NoPen);
      painter.setBrush(axisColor(axis));
      painter.drawEllipse(label, 10, 10);
      painter.setPen(Qt::white);
      painter.drawText(QRectF(label.x() - 10, label.y() - 10, 20, 20), Qt::AlignCenter, QString("XYZ"[axis]));
    }
  }
  painter.restore();
}

AttitudeWidget::AttitudeWidget(QWidget* parent) : QWidget(parent) {
  setObjectName("attitudeView");
  setMinimumSize(240, 200);
  setMouseTracking(false);
  setCursor(Qt::OpenHandCursor);
}

void AttitudeWidget::setWorldFrame(WorldFrame frame) {
  frame_ = frame;
  update();
}

void AttitudeWidget::setQuaternion(double x, double y, double z, double w) {
  Eigen::Quaterniond q(w, x, y, z);
  if (!(q.norm() > 1e-9) || !q.coeffs().allFinite()) return;
  const Eigen::Matrix3d rotation = q.normalized().toRotationMatrix();
  // stay relative to NED so rpy reads as the sensor reports it
  body_to_ned_ = frame_ == WorldFrame::Ned ? rotation : Eigen::Matrix3d(nedToEnu() * rotation);
  has_attitude_ = true;
  update();
}

void AttitudeWidget::clearAttitude() {
  has_attitude_ = false;
  body_to_ned_.setIdentity();
  update();
}

void AttitudeWidget::setViewPreset(int preset) {
  camera_.distance = kDefaultDistance;
  camera_.azimuth_deg = kDefaultAzimuth;
  camera_.elevation_deg = preset == 1 ? 2.0 : preset == 2 ? 88.0 : kDefaultElevation;
  update();
}

void AttitudeWidget::setFollowHeading(bool follow) {
  follow_heading_ = follow;
  update();
}

void AttitudeWidget::paintEvent(QPaintEvent*) {
  const auto& t = theme();
  QPainter painter(this);
  painter.setRenderHint(QPainter::Antialiasing);
  painter.fillRect(rect(), t.window);
  const Eigen::Matrix3d body_to_world = nedToEnu() * body_to_ned_;
  SceneCamera camera = camera_;
  if (follow_heading_) {
    const Eigen::Vector3d forward = body_to_world.col(0);
    const double heading = std::atan2(forward.y(), forward.x()) * 180.0 / M_PI;
    camera.azimuth_deg = heading + 180.0 + 35.0 + (camera_.azimuth_deg - kDefaultAzimuth);
  }
  const QRectF scene = QRectF(rect()).adjusted(0, 34, 0, -28);
  SceneOptions options;
  paintSensorScene(painter, scene, body_to_world, camera, options);

  const Eigen::Vector3d euler = eulerDegrees();
  const char* names[] = {"ROLL", "PITCH", "YAW"};
  const double column = std::min(110.0, width() / 3.0);
  for (int i = 0; i < 3; ++i) {
    const QRectF cell(12 + i * column, 6, column, 30);
    painter.setFont(strongFont(11));
    painter.setPen(t.muted);
    painter.drawText(cell.adjusted(0, 0, 0, -16), Qt::AlignLeft | Qt::AlignTop, names[i]);
    painter.setFont(figuresFont(19));
    painter.setPen(has_attitude_ ? t.text : t.muted);
    painter.drawText(cell.adjusted(0, 11, 0, 4), Qt::AlignLeft | Qt::AlignVCenter,
                     has_attitude_ ? QString("%1%2°").arg(euler(i) >= 0 ? "+" : "").arg(euler(i), 0, 'f', 1)
                                   : QString("--"));
  }
  if (!has_attitude_) {
    painter.setFont(strongFont(14));
    painter.setPen(t.muted);
    painter.drawText(scene, Qt::AlignHCenter | Qt::AlignBottom, "Waiting for attitude");
  }

  const QRectF strip(0, height() - 26, width(), 26);
  painter.fillRect(strip, t.ground);
  painter.setPen(t.border);
  painter.drawLine(strip.topLeft(), strip.topRight());
  struct Hint {
    const char* key;
    const char* action;
  };
  const Hint hints[] = {{"DRAG", "orbit"}, {"SCROLL", "zoom"}, {"DOUBLE-CLICK", "reset"}};
  double x = 12;
  for (const auto& hint : hints) {
    painter.setFont(strongFont(11));
    painter.setPen(t.text);
    const double key_width = QFontMetrics(strongFont(11)).horizontalAdvance(hint.key);
    painter.drawText(QRectF(x, strip.top(), key_width + 2, strip.height()), Qt::AlignVCenter, hint.key);
    x += key_width + 5;
    painter.setFont(uiFont(12));
    painter.setPen(t.muted);
    const double action_width = QFontMetrics(uiFont(12)).horizontalAdvance(hint.action);
    painter.drawText(QRectF(x, strip.top(), action_width + 2, strip.height()), Qt::AlignVCenter, hint.action);
    x += action_width + 16;
  }
  painter.setFont(uiFont(12));
  painter.setPen(t.muted);
  painter.drawText(strip.adjusted(0, 0, -12, 0), Qt::AlignRight | Qt::AlignVCenter,
                   frame_ == WorldFrame::Ned ? "sensor quaternion, NED" : "ROS driver, ENU");
}

void AttitudeWidget::mousePressEvent(QMouseEvent* event) {
  drag_origin_ = event->pos();
  drag_camera_ = camera_;
  setCursor(Qt::ClosedHandCursor);
}

void AttitudeWidget::mouseMoveEvent(QMouseEvent* event) {
  if (!(event->buttons() & Qt::LeftButton)) return;
  const QPoint delta = event->pos() - drag_origin_;
  if (delta.manhattanLength() < 3) return;
  camera_.azimuth_deg = drag_camera_.azimuth_deg - delta.x() * 0.45;
  camera_.elevation_deg = std::clamp(drag_camera_.elevation_deg + delta.y() * 0.35, -8.0, 88.0);
  update();
  emit orbited();
}

void AttitudeWidget::mouseDoubleClickEvent(QMouseEvent*) {
  setViewPreset(0);
}

void AttitudeWidget::wheelEvent(QWheelEvent* event) {
  camera_.distance = std::clamp(camera_.distance * std::pow(0.9, event->angleDelta().y() / 120.0), 3.2, 14.0);
  update();
}

namespace {

const char* quantityUnit(int quantity) {
  switch (quantity) {
    case PlotWidget::Attitude: return "deg";
    case PlotWidget::Acceleration: return "m/s²";
    case PlotWidget::Magnetic: return "G";
    default: return "rad/s";
  }
}

double niceStep(double range, int target_ticks) {
  const double raw = range / std::max(1, target_ticks);
  const double magnitude = std::pow(10.0, std::floor(std::log10(raw)));
  for (double factor : {1.0, 2.0, 2.5, 5.0, 10.0})
    if (raw <= factor * magnitude) return factor * magnitude;
  return 10.0 * magnitude;
}

QString elapsedText(double seconds) {
  if (seconds < 60) return QString("%1 s").arg(seconds, 0, 'f', 0);
  return QString("%1:%2").arg(static_cast<int>(seconds) / 60).arg(static_cast<int>(seconds) % 60, 2, 10, QChar('0'));
}

QString signedNumber(double value, int decimals) {
  return QString("%1%2").arg(value >= 0 ? "+" : "−").arg(std::abs(value), 0, 'f', decimals);
}

}  // namespace

Eigen::Vector3d telemetryEulerDeg(const Telemetry& sample, AttitudeWidget::WorldFrame frame) {
  Eigen::Quaterniond q(sample.qw, sample.qx, sample.qy, sample.qz);
  if (!(q.norm() > 1e-9) || !q.coeffs().allFinite()) return Eigen::Vector3d::Constant(std::nan(""));
  const Eigen::Matrix3d rotation = q.normalized().toRotationMatrix();
  return eulerDeg(frame == AttitudeWidget::WorldFrame::Ned ? rotation : Eigen::Matrix3d(nedToEnu() * rotation));
}

PlotWidget::PlotWidget(QWidget* parent) : QWidget(parent) {
  setObjectName("signalPlot");
  setMinimumHeight(140);
  setMouseTracking(true);
}

QStringList PlotWidget::axisNames(int quantity) {
  return quantity == Attitude ? QStringList{"Roll", "Pitch", "Yaw"} : QStringList{"X", "Y", "Z"};
}

PlotWidget::Point PlotWidget::pointFor(const Telemetry& sample) {
  Point point{sample.time_s, {}};
  const Eigen::Vector3d euler = telemetryEulerDeg(sample, frame_);
  if (euler.allFinite()) {
    // unwrapped, so yaw drifting through +-180 stays one continuous line
    unwrapped_ = last_euler_ ? Eigen::Vector3d(unwrapped_ + (euler - *last_euler_).unaryExpr([](double d) {
                                 return std::remainder(d, 360.0);
                               }))
                             : euler;
    last_euler_ = euler;
  }
  point.values[Attitude] = euler.allFinite() ? unwrapped_ : Eigen::Vector3d::Constant(std::nan(""));
  point.values[AngularRate] = {sample.gx, sample.gy, sample.gz};
  point.values[Acceleration] = {sample.ax, sample.ay, sample.az};
  point.values[Magnetic] = {sample.mx, sample.my, sample.mz};
  return point;
}

void PlotWidget::append(const Telemetry& sample) {
  const Point point = pointFor(sample);
  if (!recent_.empty() && point.time_s < recent_.back().time_s) clear();
  recent_.push_back(point);
  while (recent_.size() > 20000 || (recent_.size() > 2 && point.time_s - recent_.front().time_s > 25.0))
    recent_.pop_front();
  if (history_.empty() || point.time_s - history_.back().time_s >= 0.25) history_.push_back(point);
  while (history_.size() > 2 && point.time_s - history_.front().time_s > 1860.0) history_.pop_front();
  update();
}

void PlotWidget::setQuantity(int quantity) {
  quantity_ = std::clamp(quantity, 0, 3);
  update();
}

void PlotWidget::setWorldFrame(AttitudeWidget::WorldFrame frame) {
  frame_ = frame;
  clear();
}

void PlotWidget::setAxisVisible(int axis, bool visible) {
  if (axis < 0 || axis > 2) return;
  visible_[axis] = visible;
  update();
}

void PlotWidget::mousePressEvent(QMouseEvent* event) {
  for (int axis = 0; axis < 3; ++axis) {
    if (event->button() == Qt::LeftButton && legend_[axis].contains(event->pos())) {
      setAxisVisible(axis, !visible_[axis]);
      return;
    }
  }
  QWidget::mousePressEvent(event);
}

void PlotWidget::mouseMoveEvent(QMouseEvent* event) {
  const bool over =
      std::any_of(legend_.begin(), legend_.end(), [&](const QRectF& r) { return r.contains(event->pos()); });
  setCursor(over ? Qt::PointingHandCursor : Qt::ArrowCursor);
  setToolTip(over ? "Click to show or hide this axis; the scale fits the axes shown" : QString());
}

void PlotWidget::setWindowSeconds(double seconds) {
  window_s_ = std::clamp(seconds, 5.0, 1800.0);
  update();
}

void PlotWidget::clear() {
  recent_.clear();
  history_.clear();
  last_euler_.reset();
  update();
}

void PlotWidget::paintEvent(QPaintEvent*) {
  const auto& t = theme();
  QPainter painter(this);
  painter.setRenderHint(QPainter::Antialiasing);
  painter.fillRect(rect(), t.window);
  const QRectF area = QRectF(rect()).adjusted(64, 30, -14, -26);
  const auto& points = window_s_ <= 20.0 ? recent_ : history_;
  const double now = recent_.empty() ? 0.0 : recent_.back().time_s;
  const bool any_visible = visible_[0] || visible_[1] || visible_[2];

  double low = 0.0, high = 0.0;
  bool first = true;
  for (const auto& point : points) {
    if (now - point.time_s > window_s_) continue;
    for (int axis = 0; axis < 3; ++axis) {
      const double value = point.values[quantity_](axis);
      if (!visible_[axis] || !std::isfinite(value)) continue;
      low = first ? value : std::min(low, value);
      high = first ? value : std::max(high, value);
      first = false;
    }
  }
  const double minimum_range = quantity_ == Attitude ? 0.5 : quantity_ == AngularRate ? 0.02
                               : quantity_ == Acceleration ? 0.5 : 0.1;
  if (high - low < minimum_range) {
    const double middle = 0.5 * (high + low);
    low = middle - minimum_range / 2;
    high = middle + minimum_range / 2;
  }
  const double step = niceStep(high - low, 4);
  low = std::floor(low / step) * step;
  high = std::ceil(high / step) * step;
  auto y = [&](double value) { return area.bottom() - (value - low) / (high - low) * area.height(); };

  painter.setFont(figuresFont(12));
  for (double tick = low; tick <= high + step * 0.5; tick += step) {
    const double py = y(tick);
    const bool zero = std::abs(tick) < step * 1e-6;
    QColor line = zero ? t.muted : t.border;
    line.setAlphaF(zero ? 0.8 : 0.6);
    painter.setPen(QPen(line, 1));
    painter.drawLine(QPointF(area.left(), py), QPointF(area.right(), py));
    painter.setPen(t.muted);
    painter.drawText(QRectF(0, py - 8, area.left() - 8, 16), Qt::AlignRight | Qt::AlignVCenter,
                     QString::number(zero ? 0.0 : tick, 'g', 5));
  }
  auto x = [&](double time) { return area.right() - (now - time) / window_s_ * area.width(); };
  const double tick_step = niceStep(window_s_, 4);
  painter.setFont(uiFont(12));
  for (double seconds = 0; seconds <= window_s_ + 1e-6; seconds += tick_step) {
    const double px = area.right() - seconds / window_s_ * area.width();
    painter.setPen(QPen(t.border, 1));
    painter.drawLine(QPointF(px, area.bottom()), QPointF(px, area.bottom() + 4));
    painter.setPen(t.muted);
    painter.drawText(QRectF(px - 34, area.bottom() + 4, 68, 18), Qt::AlignCenter,
                     seconds == 0 ? QString("now") : "−" + elapsedText(seconds));
  }
  painter.setPen(QPen(t.text, 1.5));
  painter.drawLine(area.bottomLeft(), area.bottomRight());

  painter.save();
  painter.setClipRect(area.adjusted(0, -2, 0, 2));
  for (int axis = 0; axis < 3; ++axis) {
    if (!visible_[axis]) continue;
    QPainterPath path;
    bool started = false;
    for (const auto& point : points) {
      if (now - point.time_s > window_s_ + 1.0) continue;
      const double value = point.values[quantity_](axis);
      if (!std::isfinite(value)) {
        started = false;
        continue;
      }
      const QPointF at(x(point.time_s), y(value));
      started ? path.lineTo(at) : path.moveTo(at);
      started = true;
    }
    painter.setPen(QPen(axisColor(axis), 1.8));
    painter.setBrush(Qt::NoBrush);
    painter.drawPath(path);
  }
  painter.restore();

  // legend shows latest values, attitude wrapped to +-180
  painter.setFont(strongFont(12));
  painter.setPen(t.muted);
  painter.drawText(QRectF(0, 6, area.left() - 8, 18), Qt::AlignRight | Qt::AlignVCenter, quantityUnit(quantity_));
  const QStringList names = axisNames(quantity_);
  double lx = area.left();
  for (int axis = 0; axis < 3; ++axis) {
    // hidden axes stay in the legend, grayed, so they can be clicked back on
    const double start = lx;
    painter.setPen(Qt::NoPen);
    painter.setBrush(visible_[axis] ? axisColor(axis) : t.muted);
    painter.drawRect(QRectF(lx, 12, 14, 3));
    lx += 20;
    painter.setFont(strongFont(12));
    painter.setPen(visible_[axis] ? t.text : t.muted);
    const double name_width = QFontMetrics(strongFont(12)).horizontalAdvance(names[axis]) + 6;
    painter.drawText(QRectF(lx, 4, name_width, 20), Qt::AlignVCenter, names[axis]);
    lx += name_width;
    painter.setFont(figuresFont(13));
    double latest = recent_.empty() ? std::nan("") : recent_.back().values[quantity_](axis);
    if (quantity_ == Attitude) latest = std::remainder(latest, 360.0);
    const int decimals = quantity_ == AngularRate ? 5 : quantity_ == Attitude ? 2 : 3;
    painter.drawText(QRectF(lx, 4, 90, 20), Qt::AlignVCenter,
                     std::isfinite(latest) ? QString::number(latest, 'f', decimals) : QString("--"));
    lx += 96;
    legend_[axis] = QRectF(start - 4, 2, lx - start, 24);
  }
  if (recent_.size() < 2 || !any_visible) {
    painter.setFont(strongFont(14));
    painter.setPen(t.muted);
    painter.drawText(area, Qt::AlignCenter, !any_visible ? "All axes hidden" : "No telemetry yet");
  }
}

TelemetrySummaryWidget::TelemetrySummaryWidget(QWidget* parent) : QWidget(parent) {
  setObjectName("liveData");
  setMinimumHeight(320);
  rate_clock_.start();
}

void TelemetrySummaryWidget::updateSample(const Telemetry& sample) {
  ++samples_in_window_;
  if (rate_clock_.elapsed() >= 1000) {
    sample_rate_hz_ = 1000.0 * samples_in_window_ / rate_clock_.elapsed();
    samples_in_window_ = 0;
    rate_clock_.restart();
    emit freshnessChanged(true, sample_rate_hz_);
  }
  sample_ = sample;
  drift_.add(sample.time_s, telemetryEulerDeg(sample, frame_), {sample.gx, sample.gy, sample.gz});
  if (stale_) {
    stale_ = false;
    emit freshnessChanged(true, sample_rate_hz_);
  }
  update();
}

void TelemetrySummaryWidget::markStale(bool stale) {
  if (!stale || stale_) return;
  stale_ = true;
  emit freshnessChanged(false, sample_rate_hz_);
  update();
}

void TelemetrySummaryWidget::setWorldFrame(AttitudeWidget::WorldFrame frame) {
  frame_ = frame;
  drift_.reset();
  update();
}

void TelemetrySummaryWidget::setSensorRate(double rate_hz, const QString& source) {
  sensor_rate_hz_ = rate_hz;
  rate_source_ = source;
  setToolTip(source);
  update();
}

void TelemetrySummaryWidget::restartDrift() {
  drift_.reset();
  update();
}

void TelemetrySummaryWidget::paintEvent(QPaintEvent*) {
  const auto& t = theme();
  QPainter painter(this);
  painter.setRenderHint(QPainter::Antialiasing);
  painter.fillRect(rect(), t.window);
  const double left = 12, right = width() - 12;
  const double label_width = 96, unit_width = 38;
  const double column = (right - left - label_width - unit_width) / 4.0;
  const QColor value_color = stale_ ? t.muted : t.text;

  double y = 8;
  painter.setFont(strongFont(13));
  painter.setPen(t.text);
  painter.drawText(QRectF(left, y, label_width, 20), Qt::AlignVCenter, "Quantity");
  const char* heads[] = {"X", "Y", "Z", "|v|"};
  for (int i = 0; i < 4; ++i) {
    const QRectF cell(left + label_width + i * column, y, column, 20);
    painter.setPen(i < 3 ? axisColor(i) : t.text);
    painter.drawText(cell, Qt::AlignRight | Qt::AlignVCenter, heads[i]);
  }
  y += 24;
  painter.setPen(QPen(t.text, 2));
  painter.drawLine(QPointF(left, y), QPointF(right, y));

  struct Row {
    const char* label;
    Eigen::Vector3d value;
    const char* unit;
    int decimals;
  };
  Eigen::Vector3d accel = Eigen::Vector3d::Zero(), gyro = accel, mag = accel;
  if (sample_) {
    accel = {sample_->ax, sample_->ay, sample_->az};
    gyro = {sample_->gx, sample_->gy, sample_->gz};
    mag = {sample_->mx, sample_->my, sample_->mz};
  }
  const Row rows[] = {{"Acceleration", accel, "m/s²", 3}, {"Angular rate", gyro, "rad/s", 4},
                      {"Magnetic", mag, "G", 3}};
  for (const auto& row : rows) {
    const QRectF line(left, y + 2, right - left, 30);
    painter.setFont(uiFont(14));
    painter.setPen(t.text);
    painter.drawText(QRectF(line.left(), line.top(), label_width, line.height()), Qt::AlignVCenter, row.label);
    painter.setFont(figuresFont(16));
    painter.setPen(value_color);
    for (int i = 0; i < 4; ++i) {
      const double value = i < 3 ? row.value(i) : row.value.norm();
      const QRectF cell(left + label_width + i * column, line.top(), column, line.height());
      painter.drawText(cell, Qt::AlignRight | Qt::AlignVCenter,
                       sample_ ? QString::number(value, 'f', row.decimals) : QString("--"));
    }
    painter.setFont(uiFont(12));
    painter.setPen(t.muted);
    painter.drawText(QRectF(right - unit_width + 8, line.top(), unit_width - 8, line.height()),
                     Qt::AlignLeft | Qt::AlignVCenter, row.unit);
    y += 32;
    painter.setPen(QPen(t.border, 1));
    painter.drawLine(QPointF(left, y), QPointF(right, y));
  }

  y += 8;
  struct Figure {
    const char* label;
    QString value;
  };
  const Figure figures[] = {
      {"TEMPERATURE", sample_ ? QString("%1 °C").arg(sample_->temperature_c, 0, 'f', 2) : QString("--")},
      {"PRESSURE", sample_ ? QString("%1 kPa").arg(sample_->pressure_kpa, 0, 'f', 2) : QString("--")},
      {"SENSOR OUTPUT", sensor_rate_hz_ > 0 ? QString("%1 Hz").arg(sensor_rate_hz_, 0, 'g', 4) : QString("--")},
      {"VIEW UPDATES", sample_rate_hz_ > 0 ? QString("%1 Hz").arg(sample_rate_hz_, 0, 'f', 0) : QString("--")}};
  const double figure_width = (right - left) / 4.0;
  for (int i = 0; i < 4; ++i) {
    const QRectF cell(left + i * figure_width, y, figure_width, 42);
    painter.setFont(strongFont(11));
    painter.setPen(t.muted);
    painter.drawText(cell, Qt::AlignLeft | Qt::AlignTop, figures[i].label);
    painter.setFont(figuresFont(19));
    painter.setPen(i == 2 ? t.text : value_color);
    painter.drawText(cell.adjusted(0, 14, 0, 0), Qt::AlignLeft | Qt::AlignVCenter, figures[i].value);
  }
  y += 50;

  painter.setPen(QPen(t.text, 2));
  painter.drawLine(QPointF(left, y), QPointF(right, y));
  y += 4;
  painter.setFont(strongFont(13));
  painter.setPen(t.text);
  painter.drawText(QRectF(left, y, 200, 22), Qt::AlignVCenter, "Drift while still");
  const auto rate = drift_.rateDegPerMinute();
  QString state;
  StatusChip::Level level = StatusChip::Level::Neutral;
  if (!drift_.hasData() || stale_) {
    state = "No attitude";
  } else if (drift_.moving()) {
    state = "Moving";
    level = StatusChip::Level::Warn;
  } else if (!rate) {
    const double remaining = std::max(0.0, drift_.minimumStillSeconds() - drift_.stillSeconds());
    state = QString("Settling %1 s").arg(remaining, 0, 'f', 0);
    level = StatusChip::Level::Info;
  } else {
    state = "Measuring";
    level = StatusChip::Level::Ok;
  }
  const double chip_width = statusChipWidth(state);
  paintStatusChip(painter, QRectF(right - chip_width, y + 1, chip_width, 20), state, level);
  y += 26;
  const char* names[] = {"Roll", "Pitch", "Yaw"};
  const double drift_column = (right - left - label_width - unit_width) / 3.0;
  painter.setFont(strongFont(12));
  for (int i = 0; i < 3; ++i) {
    painter.setPen(t.muted);
    painter.drawText(QRectF(left + label_width + i * drift_column, y, drift_column, 18),
                     Qt::AlignRight | Qt::AlignVCenter, names[i]);
  }
  y += 18;
  const auto since = drift_.sinceMark();
  const std::pair<QString, std::optional<Eigen::Vector3d>> drift_rows[] = {
      {"Rate", rate}, {QString("Total, %1").arg(elapsedText(drift_.markSeconds())), since}};
  const char* drift_units[] = {"°/min", "°"};
  for (int row = 0; row < 2; ++row) {
    painter.setFont(uiFont(14));
    painter.setPen(t.text);
    painter.drawText(QRectF(left, y, label_width + 10, 28), Qt::AlignVCenter, drift_rows[row].first);
    painter.setFont(figuresFont(16));
    for (int i = 0; i < 3; ++i) {
      const auto& value = drift_rows[row].second;
      // yaw usually walks, so full color
      painter.setPen(stale_ || !value ? t.muted : i == 2 ? t.text : blend(t.muted, t.text, 0.6));
      painter.drawText(QRectF(left + label_width + i * drift_column, y, drift_column, 28),
                       Qt::AlignRight | Qt::AlignVCenter, value ? signedNumber((*value)(i), row == 0 ? 3 : 2) : "--");
    }
    painter.setFont(uiFont(12));
    painter.setPen(t.muted);
    painter.drawText(QRectF(right - unit_width + 8, y, unit_width - 8, 28), Qt::AlignLeft | Qt::AlignVCenter,
                     drift_units[row]);
    y += 28;
  }
  painter.setFont(uiFont(12));
  painter.setPen(t.muted);
  const QString note = drift_.movedSinceMark()
      ? "The sensor moved since the restart, so the total includes real motion."
      : QString("Rate: straight-line fit over the last %1 s still. Any motion restarts it.")
            .arg(drift_.windowSeconds(), 0, 'f', 0);
  painter.drawText(QRectF(left, y, right - left, 18), Qt::AlignLeft | Qt::AlignVCenter,
                   painter.fontMetrics().elidedText(note, Qt::ElideRight, right - left));
}

BandwidthMeter::BandwidthMeter(QWidget* parent) : QWidget(parent) {
  setObjectName("bandwidthMeter");
  setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
}

void BandwidthMeter::setEstimates(const BandwidthEstimate& device, const std::optional<BandwidthEstimate>& staged) {
  device_ = device;
  staged_ = staged;
  QStringList lines;
  auto describe = [&](const QString& title, const BandwidthEstimate& estimate) {
    lines << QString("<b>%1</b>").arg(title);
    if (!estimate.known) {
      lines << "Not known yet: read register " + (estimate.missing.isEmpty() ? QString("5") : estimate.missing);
      return;
    }
    lines << QString("%1 of %2 baud (%3 bit/s)")
                 .arg(QString::number(100 * estimate.fraction(), 'f', 0) + "%")
                 .arg(estimate.baud).arg(estimate.bits_per_second, 0, 'f', 0);
    for (const auto& part : estimate.parts) lines << "&nbsp;&nbsp;" + part.toHtmlEscaped();
    if (estimate.parts.isEmpty()) lines << "&nbsp;&nbsp;No async or binary output on this port";
    for (const auto& note : estimate.notes) lines << "&nbsp;&nbsp;<i>" + note.toHtmlEscaped() + "</i>";
  };
  describe("Serial port 1 load, sensor now", device_);
  if (staged_) describe("With staged changes applied", *staged_);
  lines << "<br>Every byte is 10 bits on the line. Over 100% the sensor drops output, so applying or saving such "
           "a configuration is blocked.";
  setToolTip(lines.join("<br>"));
  update();
}

void BandwidthMeter::paintEvent(QPaintEvent*) {
  const auto& t = theme();
  QPainter painter(this);
  painter.setRenderHint(QPainter::Antialiasing);
  auto color = [&](const BandwidthEstimate& e) {
    if (!e.known) return t.muted;
    return e.fraction() > 1.0 ? t.error : e.fraction() > 0.8 ? t.warn : t.ok;
  };
  auto percent = [](const BandwidthEstimate& e) {
    return e.known ? QString::number(100 * e.fraction(), 'f', 0) + "%" : QString("?");
  };
  painter.setFont(strongFont(11));
  painter.setPen(t.muted);
  painter.drawText(QRectF(0, 0, 70, height()), Qt::AlignLeft | Qt::AlignVCenter, "LINE LOAD");
  const QRectF track(70, height() / 2.0 - 5, 84, 10);
  painter.setPen(QPen(t.border, 1));
  painter.setBrush(t.window);
  painter.drawRoundedRect(track, 2, 2);
  painter.setPen(Qt::NoPen);
  const BandwidthEstimate& shown = staged_ ? *staged_ : device_;
  if (device_.known) {
    painter.setBrush(color(device_));
    painter.drawRoundedRect(QRectF(track.left(), track.top(), track.width() * std::min(1.0, device_.fraction()),
                                   track.height()), 2, 2);
  }
  if (staged_ && staged_->known) {
    painter.setBrush(Qt::NoBrush);
    painter.setPen(QPen(color(*staged_), 2));
    const double w = track.width() * std::min(1.0, staged_->fraction());
    painter.drawRoundedRect(QRectF(track.left(), track.top() - 2, std::max(w, 3.0), track.height() + 4), 2, 2);
  }
  painter.setFont(figuresFont(16));
  QString text = percent(device_);
  if (staged_) text += " → " + percent(*staged_);
  painter.setPen(color(shown));
  painter.drawText(QRectF(track.right() + 8, 0, width() - track.right() - 8, height()),
                   Qt::AlignLeft | Qt::AlignVCenter, text);
}

}  // namespace riptide_imu
