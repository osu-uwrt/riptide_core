#include "riptide_imu/config_tool/theme.hpp"

#include <QApplication>
#include <QFontDatabase>
#include <QPalette>
#include <QStyleFactory>
#include <QWidget>
#include <algorithm>
#include <map>
#include <vector>

namespace riptide_imu {
namespace {

// Values from nereus/content/viewer/themes/heat-sheet.yaml; the bars use its board block.
ThemeColors heatSheet() {
  ThemeColors t;
  t.id = "heat-sheet";
  t.label = "Heat sheet";
  t.window = QColor("#ffffff");
  t.ground = QColor("#e6e9ed");
  t.frame = QColor("#f6f7f9");
  t.frame_hovered = QColor("#eef1f5");
  t.button = QColor("#eef0f3");
  t.button_hovered = QColor("#e1e5ea");
  t.button_pressed = QColor("#d2d8df");
  t.border = QColor("#c4cad1");
  t.header = QColor("#dce6f6");
  t.scroll_grab = QColor("#c4cad1");
  t.row_alt = QColor(0x15, 0x18, 0x1c, 7);
  t.tab_overline = QColor("#15181c");
  t.text = QColor("#15181c");
  t.muted = QColor("#5b636d");
  t.accent = QColor("#1f5fbf");
  t.active = QColor("#1f5fbf");
  t.active_hovered = QColor("#2a6bce");
  t.active_pressed = QColor("#1a52a6");
  t.active_text = QColor("#ffffff");
  t.danger = QColor("#d22b2b");
  t.danger_hovered = QColor("#e03a3a");
  t.warn = QColor("#935400");
  t.error = QColor("#c62828");
  t.ok = QColor("#12813a");
  t.change = QColor("#ffd166");
  t.change_text = QColor("#15181c");
  t.board = QColor("#15181c");
  t.board_text = QColor("#f3f5f7");
  t.board_muted = QColor("#a3acb6");
  t.board_accent = QColor("#8fb8ff");
  t.board_button = QColor("#2c2f33");
  t.board_button_hovered = QColor("#3d3f43");
  t.board_frame = QColor("#25282c");
  t.board_border = QColor("#444649");
  t.board_warn = QColor("#ffb21a");
  t.board_ok = QColor("#4ade80");
  t.series = {QColor("#2a78d6"), QColor("#e0591f"), QColor("#12936a"), QColor("#b07800"),
              QColor("#cc5a8a"), QColor("#008300"), QColor("#4a3aa7"), QColor("#d93f3e")};
  t.axis_x = QColor("#d93f3e");
  t.axis_y = QColor("#12936a");
  t.axis_z = QColor("#2a78d6");
  return t;
}

// nereus/content/viewer/themes/timing-board.yaml: the heat sheet after dark.
ThemeColors timingBoard() {
  ThemeColors t;
  t.id = "timing-board";
  t.label = "Timing board";
  t.dark = true;
  t.window = QColor("#15181c");
  t.ground = QColor("#0c0e11");
  t.frame = QColor("#1d2126");
  t.frame_hovered = QColor("#242930");
  t.button = QColor("#22272e");
  t.button_hovered = QColor("#2c323a");
  t.button_pressed = QColor("#37404a");
  t.border = QColor("#353b44");
  t.header = QColor("#1d3256");
  t.scroll_grab = QColor("#3a414b");
  t.row_alt = QColor(255, 255, 255, 8);
  t.tab_overline = QColor("#e9ecef");
  t.text = QColor("#e9ecef");
  t.muted = QColor("#98a1ac");
  t.accent = QColor("#6ea2ff");
  t.active = QColor("#2f6fdb");
  t.active_hovered = QColor("#3d7ce6");
  t.active_pressed = QColor("#2660c4");
  t.active_text = QColor("#ffffff");
  t.danger = QColor("#d92d20");
  t.danger_hovered = QColor("#e8463a");
  t.warn = QColor("#ffb21a");
  t.error = QColor("#ff6b5f");
  t.ok = QColor("#4ade80");
  t.change = QColor("#ffb21a");
  t.change_text = QColor("#15181c");
  t.board = QColor("#050607");
  t.board_text = QColor("#f3f5f7");
  t.board_muted = QColor("#a3acb6");
  t.board_accent = QColor("#8fb8ff");
  t.board_button = QColor("#1e1f20");
  t.board_button_hovered = QColor("#303031");
  t.board_frame = QColor("#171718");
  t.board_border = QColor("#373839");
  t.board_warn = QColor("#ffb21a");
  t.board_ok = QColor("#4ade80");
  t.series = {QColor("#3987e5"), QColor("#d95926"), QColor("#199e70"), QColor("#c98500"),
              QColor("#d55181"), QColor("#008300"), QColor("#9085e9"), QColor("#e66767")};
  t.axis_x = QColor("#e66767");
  t.axis_y = QColor("#199e70");
  t.axis_z = QColor("#3987e5");
  return t;
}

ThemeColors& current() {
  static ThemeColors colors = timingBoard();
  return colors;
}

QString rgba(const QColor& c) {
  return QString("rgba(%1,%2,%3,%4)").arg(c.red()).arg(c.green()).arg(c.blue()).arg(c.alpha());
}

// The style sheet, written once with @tokens for the theme's colors.
QString styleSheet(const ThemeColors& t, const QString& share_directory) {
  QString sheet = R"(
* { font-family: "Barlow TF"; font-size: 14px; }
QMainWindow, QDialog { background: @ground; }
QWidget { color: @text; }
QToolTip { background: @board; color: @board_text; border: 1px solid @board_border; padding: 4px 6px; }

QMenuBar { background: @board; color: @board_text; padding: 2px 4px; }
QMenuBar::item { background: transparent; padding: 4px 10px; }
QMenuBar::item:selected { background: @board_button_hovered; }
QMenu { background: @window; border: 1px solid @border; padding: 4px; }
QMenu::item { padding: 5px 24px 5px 12px; }
QMenu::item:selected { background: @header; color: @text; }
QMenu::item:disabled { color: @muted; }
QMenu::separator { height: 1px; background: @border; margin: 4px 6px; }

QWidget#board { background: @board; }
QWidget#board QLabel { color: @board_text; }
QWidget#board QLabel[tone="muted"] { color: @board_muted; }
QLabel#wordmark { font-family: "Barlow TF Semi Condensed"; font-weight: bold; font-size: 21px; color: @board_text; }
QWidget#board QPushButton { background: @board_button; color: @board_text; border: 1px solid @board_border; }
QWidget#board QPushButton:hover { background: @board_button_hovered; }
QWidget#board QPushButton:disabled { color: @board_muted; }
QWidget#board QPushButton[role="primary"] { background: @active; border-color: @active; color: @active_text; }
QWidget#board QPushButton[role="primary"]:hover { background: @active_hovered; }
QWidget#board QComboBox { background: @board_frame; color: @board_text; border: 1px solid @board_border; }
QWidget#board QComboBox:disabled { color: @board_muted; }
QWidget#board QComboBox QAbstractItemView { background: @board_frame; color: @board_text;
  selection-background-color: @board_button_hovered; selection-color: @board_text; }

QWidget#commandBar, QWidget#wizardFooter { background: @ground; border-bottom: 1px solid @border; }
QWidget#wizardFooter { border-bottom: 0; border-top: 1px solid @border; }
QWidget#sheet, QStackedWidget#wizardPages, QWidget#wizardPage { background: @window; }

QPushButton { background: @button; border: 1px solid @border; border-radius: 2px; padding: 5px 12px; color: @text; }
QPushButton:hover { background: @button_hovered; }
QPushButton:pressed { background: @button_pressed; }
QPushButton:disabled { color: @muted; background: @frame; }
QPushButton:checked { background: @active; border-color: @active; color: @active_text; }
QPushButton[role="primary"] { background: @active; border-color: @active; color: @active_text; font-weight: 600; }
QPushButton[role="primary"]:hover { background: @active_hovered; }
QPushButton[role="primary"]:pressed { background: @active_pressed; }
QPushButton[role="primary"]:disabled { background: @frame; border-color: @border; color: @muted; font-weight: normal; }
QPushButton[role="danger"] { background: @danger; border-color: @danger; color: #ffffff; font-weight: 600; }
QPushButton[role="danger"]:hover { background: @danger_hovered; }
QPushButton[role="quiet"] { background: transparent; border-color: transparent; color: @muted; }
QPushButton[role="quiet"]:hover { background: @button_hovered; color: @text; }

QLineEdit, QComboBox, QDoubleSpinBox, QSpinBox, QPlainTextEdit {
  background: @frame; border: 1px solid @border; border-radius: 2px; padding: 4px 6px;
  selection-background-color: @accent; selection-color: @active_text; }
QLineEdit:focus, QComboBox:focus, QDoubleSpinBox:focus, QPlainTextEdit:focus { border-color: @accent; }
QComboBox QAbstractItemView { background: @window; border: 1px solid @border;
  selection-background-color: @header; selection-color: @text; outline: 0; }
QPlainTextEdit#protocolLog { font-family: "DejaVu Sans Mono", monospace; font-size: 12px; background: @window; border: 0; }

QTabWidget::pane { border: 0; background: @window; }
QTabWidget::tab-bar { left: 0; }
QTabBar { background: @ground; }
QTabBar::tab { background: @ground; color: @muted; padding: 7px 16px 6px 16px; border: 0;
  border-top: 2px solid transparent; margin-right: 1px; }
QTabBar::tab:selected { background: @window; color: @text; border-top: 2px solid @tab_overline; font-weight: 600; }
QTabBar::tab:hover:!selected { background: @button_hovered; color: @text; }

QTreeView, QTableView, QListView { background: @window; alternate-background-color: @row_alt; border: 0;
  gridline-color: @border; selection-background-color: @header; selection-color: @text; outline: 0; }
QTreeView::item, QTableView::item { padding: 3px 2px; border: 0; }
QTreeView::item:selected, QTableView::item:selected { background: @header; color: @text; }
QTreeView::item:hover:!selected { background: @frame_hovered; }
QHeaderView { background: @window; border: 0; }
QHeaderView::section { background: @window; color: @text; font-weight: 600; border: 0;
  border-bottom: 2px solid @text; padding: 5px 6px; }
QTableCornerButton::section { background: @window; border: 0; }

QScrollBar:vertical { background: transparent; width: 10px; margin: 0; }
QScrollBar:horizontal { background: transparent; height: 10px; margin: 0; }
QScrollBar::handle { background: @scroll_grab; border-radius: 2px; min-height: 24px; min-width: 24px; }
QScrollBar::handle:hover { background: @muted; }
QScrollBar::add-line, QScrollBar::sub-line { width: 0; height: 0; }
QScrollBar::add-page, QScrollBar::sub-page { background: none; }

QSplitter::handle { background: @ground; }
QSplitter::handle:horizontal { width: 6px; }
QSplitter::handle:vertical { height: 6px; }

QLabel#sectionTitle { font-weight: 600; font-size: 15px; padding: 2px 0 4px 0; border-bottom: 2px solid @text; }
QLabel#pageTitle { font-family: "Barlow TF Semi Condensed"; font-weight: bold; font-size: 26px; }
QLabel[tone="muted"] { color: @muted; }
QLabel[tone="warn"] { color: @warn; }
QLabel[tone="error"] { color: @error; }
QLabel[tone="ok"] { color: @ok; }
QLabel#hint { color: @muted; font-size: 13px; }
QFrame#resultCard { background: @frame; border: 1px solid @border; border-left: 3px solid @accent; border-radius: 2px; }
QFrame#resultCard QLabel { background: transparent; }

QCheckBox { spacing: 8px; }
QCheckBox::indicator { width: 16px; height: 16px; border: 1px solid @border; border-radius: 2px; background: @frame; }
QCheckBox::indicator:checked { background: @active; border-color: @active; image: url(@icons/check.svg); }
QCheckBox::indicator:hover { border-color: @accent; }

QStatusBar { background: @ground; color: @muted; border-top: 1px solid @border; }
QStatusBar::item { border: 0; }
QStatusBar QLabel { padding: 3px 8px; }
QStatusBar QLabel[tone="error"] { color: @error; }

QMessageBox, QFileDialog { background: @window; }
)";
  const std::map<QString, QColor> tokens{
      {"window", t.window}, {"ground", t.ground}, {"frame_hovered", t.frame_hovered}, {"frame", t.frame},
      {"button_hovered", t.button_hovered}, {"button_pressed", t.button_pressed}, {"button", t.button},
      {"border", t.border}, {"header", t.header}, {"scroll_grab", t.scroll_grab}, {"row_alt", t.row_alt},
      {"tab_overline", t.tab_overline}, {"text", t.text}, {"muted", t.muted}, {"accent", t.accent},
      {"active_hovered", t.active_hovered}, {"active_pressed", t.active_pressed},
      {"active_text", t.active_text}, {"active", t.active}, {"danger_hovered", t.danger_hovered},
      {"danger", t.danger}, {"warn", t.warn}, {"error", t.error}, {"ok", t.ok},
      {"board_text", t.board_text}, {"board_muted", t.board_muted},
      {"board_button_hovered", t.board_button_hovered}, {"board_button", t.board_button},
      {"board_frame", t.board_frame}, {"board_border", t.board_border}, {"board", t.board}};
  // Longest names first so "@frame_hovered" is not consumed by "@frame".
  std::vector<std::pair<QString, QColor>> ordered(tokens.begin(), tokens.end());
  std::sort(ordered.begin(), ordered.end(),
            [](const auto& a, const auto& b) { return a.first.size() > b.first.size(); });
  sheet.replace("@icons", share_directory + "/icons");
  for (const auto& [name, color] : ordered) sheet.replace("@" + name, rgba(color));
  return sheet;
}

QString& loadedShare() {
  static QString directory;
  return directory;
}

void loadFonts(const QString& share_directory) {
  if (!loadedShare().isEmpty() || share_directory.isEmpty()) return;
  loadedShare() = share_directory;
  for (const char* file : {"BarlowTF-Regular.ttf", "BarlowTF-Medium.ttf", "BarlowTF-SemiBold.ttf",
                           "BarlowTF-SemiCondensedBold.ttf"})
    QFontDatabase::addApplicationFont(share_directory + "/fonts/" + file);
}

}  // namespace

QStringList themeIds() { return {"timing-board", "heat-sheet"}; }

QString themeLabel(const QString& id) {
  return id == "heat-sheet" ? heatSheet().label : timingBoard().label;
}

void applyTheme(QApplication* application, const QString& id, const QString& share_directory) {
  loadFonts(share_directory);
  current() = id == "heat-sheet" ? heatSheet() : timingBoard();
  const ThemeColors& t = current();
  application->setStyle(QStyleFactory::create("Fusion"));
  QPalette palette;
  palette.setColor(QPalette::Window, t.window);
  palette.setColor(QPalette::WindowText, t.text);
  palette.setColor(QPalette::Base, t.frame);
  palette.setColor(QPalette::AlternateBase, t.window);
  palette.setColor(QPalette::ToolTipBase, t.board);
  palette.setColor(QPalette::ToolTipText, t.board_text);
  palette.setColor(QPalette::Text, t.text);
  palette.setColor(QPalette::Button, t.button);
  palette.setColor(QPalette::ButtonText, t.text);
  palette.setColor(QPalette::BrightText, t.error);
  palette.setColor(QPalette::Highlight, t.header);
  palette.setColor(QPalette::HighlightedText, t.text);
  palette.setColor(QPalette::Link, t.accent);
  palette.setColor(QPalette::PlaceholderText, t.muted);
  palette.setColor(QPalette::Disabled, QPalette::Text, t.muted);
  palette.setColor(QPalette::Disabled, QPalette::ButtonText, t.muted);
  palette.setColor(QPalette::Disabled, QPalette::WindowText, t.muted);
  application->setPalette(palette);
  application->setFont(uiFont());
  application->setStyleSheet(styleSheet(t, loadedShare()));
  application->setProperty("vnTheme", t.id);
  for (QWidget* widget : QApplication::allWidgets()) widget->update();
}

const ThemeColors& theme() { return current(); }

QFont uiFont(int pixel_size) {
  QFont font("Barlow TF");
  font.setPixelSize(pixel_size);
  return font;
}

QFont strongFont(int pixel_size) {
  QFont font("Barlow TF");
  font.setPixelSize(pixel_size);
  font.setWeight(QFont::DemiBold);
  return font;
}

QFont figuresFont(int pixel_size) {
  QFont font("Barlow TF Semi Condensed");
  font.setPixelSize(pixel_size);
  font.setWeight(QFont::Bold);
  return font;
}

QColor axisColor(int axis) {
  const auto& t = theme();
  return axis == 0 ? t.axis_x : axis == 1 ? t.axis_y : t.axis_z;
}

QColor blend(const QColor& a, const QColor& b, double t) {
  return QColor::fromRgbF(a.redF() + (b.redF() - a.redF()) * t, a.greenF() + (b.greenF() - a.greenF()) * t,
                          a.blueF() + (b.blueF() - a.blueF()) * t, a.alphaF() + (b.alphaF() - a.alphaF()) * t);
}

}  // namespace riptide_imu
