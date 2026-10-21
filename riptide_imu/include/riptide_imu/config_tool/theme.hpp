#pragma once

#include <QColor>
#include <QFont>
#include <QString>
#include <QStringList>
#include <array>

class QApplication;

namespace riptide_imu {

// The Nereus viewer's "Heat sheet" / "Timing board" world (nereus/content/viewer/themes): white or charcoal sheets
// on a rule-gray ground, an ink board for the bars, ruled heads, lane blue for selection and "on".
struct ThemeColors {
  QString id;
  QString label;
  bool dark{false};
  // sheets and controls
  QColor window, ground, frame, frame_hovered, button, button_hovered, button_pressed, border, header,
      scroll_grab, row_alt, tab_overline;
  // text and status
  QColor text, muted, accent, active, active_hovered, active_pressed, active_text, danger, danger_hovered,
      warn, error, ok, change, change_text;
  // the board (menu and command bars)
  QColor board, board_text, board_muted, board_accent, board_button, board_button_hovered, board_frame,
      board_border, board_warn, board_ok;
  // plot lines, and the X / Y / Z colors drawn everywhere an axis is shown
  std::array<QColor, 8> series;
  QColor axis_x, axis_y, axis_z;
};

// Ids accepted by applyTheme(), in menu order (the first is the default).
QStringList themeIds();
QString themeLabel(const QString& id);

// Loads the Barlow TF fonts from <share>/config_tool_assets/fonts, then styles the application. Unknown ids fall back to the default.
void applyTheme(QApplication* application, const QString& id, const QString& share_directory = QString());
const ThemeColors& theme();

// The text face, the strong (semibold) face and the figures face (semi-condensed bold, tabular digits).
QFont uiFont(int pixel_size = 14);
QFont strongFont(int pixel_size = 14);
QFont figuresFont(int pixel_size = 14);

QColor axisColor(int axis);
QColor blend(const QColor& a, const QColor& b, double t);

}  // namespace riptide_imu
