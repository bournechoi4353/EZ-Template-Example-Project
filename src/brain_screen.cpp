#include "brain_screen.hpp"

#include <cmath>

#include "main.h"  // IWYU pragma: keep

// LVGL-based touch auton selector (liblvgl v8.3, the one PROS/EZ-Template ship).
//
//   +----------+-------+----------------------+
//   |  auton   | logo  |                      |
//   |  list    |  ^^   |     field map        |
//   | (scroll  | [UP]  |   + route line       |
//   |  + tap)  | [DN]  |   + moving square    |
//   |          | batt  |                      |
//   +----------+-------+----------------------+
//
// The right panel plays the SELECTED auton's coded path: each pid_drive_set /
// pid_turn_relative_set is integrated into (x, y, heading) using the exact
// numbers from autons.cpp, then a square walks the route. Nothing here changes
// how an auton drives -- it only draws what the code already says.
//
// All LVGL mutation happens in anim_timer() (LVGL's own thread). Other tasks
// (controller selection, autonomous) only flag dirty_.

BrainScreen brain_screen;

namespace {
// ---- layout (480 x 240) ----
constexpr int LIST_X = 6, LIST_Y = 6, LIST_W = 150, LIST_H = 228;
constexpr int MID_X = 160;
constexpr int FIELD_X = 244, FIELD_Y = 12, FIELD_PX = 216;  // square field panel
constexpr int ROBOT_SZ = 12;
constexpr double FIELD_IN = 144.0;  // VEX field is 144" across

// ---- theme (2145Z reference palette) ----
const lv_color_t THEME = lv_color_hex(0xffade7);  // pink
const lv_color_t PANEL = lv_color_hex(0x1c1c1c);
const lv_color_t WHITE = lv_color_hex(0xffffff);
const lv_color_t ROBOTC = lv_color_hex(0x00ffff);  // cyan square, easy to see

lv_style_t style_panel;
lv_style_t style_row;
bool styles_ready = false;

void init_styles() {
  if (styles_ready) return;
  styles_ready = true;

  lv_style_init(&style_panel);
  lv_style_set_bg_color(&style_panel, PANEL);
  lv_style_set_bg_opa(&style_panel, LV_OPA_COVER);
  lv_style_set_text_color(&style_panel, WHITE);
  lv_style_set_border_width(&style_panel, 0);
  lv_style_set_radius(&style_panel, 6);
  lv_style_set_text_font(&style_panel, &lv_font_montserrat_16);

  lv_style_init(&style_row);
  lv_style_set_text_color(&style_row, WHITE);
  lv_style_set_text_font(&style_row, &lv_font_montserrat_14);
  lv_style_set_border_width(&style_row, 0);
  lv_style_set_radius(&style_row, 4);
  lv_style_set_outline_color(&style_row, WHITE);
  lv_style_set_outline_width(&style_row, 0);
  lv_style_set_bg_opa(&style_row, 150);
}
}  // namespace

// ===========================================================================
// build
// ===========================================================================
void BrainScreen::init(const std::vector<AutonItem>& autons) {
  autons_ = autons;
  init_styles();

  screen_ = lv_obj_create(NULL);
  lv_obj_set_style_bg_color(screen_, lv_color_hex(0x000000), LV_PART_MAIN);
  lv_obj_clear_flag(screen_, LV_OBJ_FLAG_SCROLLABLE);

  // ---- LEFT: auton list ----
  list_ = lv_list_create(screen_);
  lv_obj_set_size(list_, LIST_W, LIST_H);
  lv_obj_set_pos(list_, LIST_X, LIST_Y);
  lv_obj_set_style_bg_color(list_, PANEL, LV_PART_MAIN);
  lv_obj_set_style_border_width(list_, 0, LV_PART_MAIN);
  lv_obj_set_style_pad_all(list_, 4, LV_PART_MAIN);
  lv_obj_set_style_pad_row(list_, 3, LV_PART_MAIN);
  lv_obj_set_scrollbar_mode(list_, LV_SCROLLBAR_MODE_ACTIVE);

  rows_.clear();
  for (size_t i = 0; i < autons_.size(); i++) {
    lv_obj_t* row = lv_list_add_btn(list_, NULL, autons_[i].name.c_str());
    lv_obj_add_style(row, &style_row, LV_PART_MAIN);
    lv_obj_set_style_bg_color(row, autons_[i].color, LV_PART_MAIN);
    lv_obj_set_style_pad_ver(row, 6, LV_PART_MAIN);
    lv_obj_add_event_cb(row, row_clicked, LV_EVENT_CLICKED, (void*)(intptr_t)i);
    rows_.push_back(row);
  }

  // ---- MIDDLE: logo (top) + scroll buttons + battery ----
  // middle column spans ~x156..244 (center ~200); logo is 44x72 from assets/logo.png
  LV_IMG_DECLARE(logo);
  lv_obj_t* logo_img = lv_img_create(screen_);
  lv_img_set_src(logo_img, &logo);
  lv_obj_set_pos(logo_img, 178, 6);

  lv_obj_t* up = lv_btn_create(screen_);
  lv_obj_add_style(up, &style_panel, LV_PART_MAIN);
  lv_obj_set_style_bg_color(up, THEME, LV_PART_MAIN);
  lv_obj_set_size(up, 56, 40);
  lv_obj_set_pos(up, 172, 88);
  lv_obj_add_event_cb(up, scroll_up, LV_EVENT_CLICKED, NULL);
  lv_obj_t* up_lbl = lv_label_create(up);
  lv_obj_set_style_text_font(up_lbl, &lv_font_montserrat_20, LV_PART_MAIN);
  lv_label_set_text(up_lbl, LV_SYMBOL_UP);
  lv_obj_center(up_lbl);

  lv_obj_t* down = lv_btn_create(screen_);
  lv_obj_add_style(down, &style_panel, LV_PART_MAIN);
  lv_obj_set_style_bg_color(down, THEME, LV_PART_MAIN);
  lv_obj_set_size(down, 56, 40);
  lv_obj_set_pos(down, 172, 134);
  lv_obj_add_event_cb(down, scroll_down, LV_EVENT_CLICKED, NULL);
  lv_obj_t* down_lbl = lv_label_create(down);
  lv_obj_set_style_text_font(down_lbl, &lv_font_montserrat_20, LV_PART_MAIN);
  lv_label_set_text(down_lbl, LV_SYMBOL_DOWN);
  lv_obj_center(down_lbl);

  battery_ = lv_label_create(screen_);
  lv_obj_set_style_text_color(battery_, WHITE, LV_PART_MAIN);
  lv_obj_set_style_text_font(battery_, &lv_font_montserrat_14, LV_PART_MAIN);
  lv_label_set_text(battery_, "--%");
  lv_obj_set_pos(battery_, MID_X + 12, 200);

  // ---- RIGHT: field map (converted from assets/field.png) ----
  LV_IMG_DECLARE(field_match);
  field_ = lv_img_create(screen_);
  lv_img_set_src(field_, &field_match);
  lv_obj_set_size(field_, FIELD_PX, FIELD_PX);
  lv_obj_set_pos(field_, FIELD_X, FIELD_Y);
  lv_obj_clear_flag(field_, LV_OBJ_FLAG_SCROLLABLE);

  // route polyline (drawn on the field)
  route_ = lv_line_create(field_);
  lv_obj_set_style_line_color(route_, WHITE, LV_PART_MAIN);
  lv_obj_set_style_line_width(route_, 2, LV_PART_MAIN);
  lv_obj_set_style_line_opa(route_, 120, LV_PART_MAIN);
  lv_obj_set_style_line_rounded(route_, true, LV_PART_MAIN);

  // the moving square
  robot_ = lv_obj_create(field_);
  lv_obj_set_size(robot_, ROBOT_SZ, ROBOT_SZ);
  lv_obj_set_style_bg_color(robot_, ROBOTC, LV_PART_MAIN);
  lv_obj_set_style_border_color(robot_, WHITE, LV_PART_MAIN);
  lv_obj_set_style_border_width(robot_, 1, LV_PART_MAIN);
  lv_obj_set_style_radius(robot_, 1, LV_PART_MAIN);
  lv_obj_clear_flag(robot_, LV_OBJ_FLAG_SCROLLABLE);

  // heading dot -- sits on the leading edge so you can see which way it faces
  nose_ = lv_obj_create(field_);
  lv_obj_set_size(nose_, 5, 5);
  lv_obj_set_style_bg_color(nose_, lv_color_hex(0xffd700), LV_PART_MAIN);  // yellow
  lv_obj_set_style_border_width(nose_, 0, LV_PART_MAIN);
  lv_obj_set_style_radius(nose_, 3, LV_PART_MAIN);
  lv_obj_clear_flag(nose_, LV_OBJ_FLAG_SCROLLABLE);

  // ---- text overlay for show_text() -- covers the field panel, hidden by default ----
  map_text_ = lv_label_create(screen_);
  lv_obj_add_style(map_text_, &style_panel, LV_PART_MAIN);
  lv_obj_set_style_bg_opa(map_text_, LV_OPA_COVER, LV_PART_MAIN);
  lv_obj_set_style_pad_all(map_text_, 6, LV_PART_MAIN);
  lv_obj_set_size(map_text_, FIELD_PX, FIELD_PX);
  lv_obj_set_pos(map_text_, FIELD_X, FIELD_Y);
  lv_label_set_long_mode(map_text_, LV_LABEL_LONG_WRAP);
  lv_obj_add_flag(map_text_, LV_OBJ_FLAG_HIDDEN);

  lv_timer_create(battery_timer, 2000, NULL);
  lv_timer_create(anim_timer, 40, NULL);  // ~25 fps

  lv_scr_load(screen_);
}

// ===========================================================================
// path integration + rendering
// ===========================================================================
lv_point_t BrainScreen::to_px(const Pose& p) const {
  lv_point_t pt;
  pt.x = (lv_coord_t)((p.x + FIELD_IN / 2) / FIELD_IN * FIELD_PX);
  pt.y = (lv_coord_t)((FIELD_IN / 2 - p.y) / FIELD_IN * FIELD_PX);  // +Y = up
  return pt;
}

void BrainScreen::rebuild_preview() {
  anim_pts_.clear();
  route_pts_.clear();
  anim_i_ = 0;

  const Preview& pv = autons_[selected_index_].preview;
  if (!pv.has || pv.moves.empty()) {
    lv_obj_add_flag(robot_, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(nose_, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(route_, LV_OBJ_FLAG_HIDDEN);
    return;
  }
  lv_obj_clear_flag(robot_, LV_OBJ_FLAG_HIDDEN);
  lv_obj_clear_flag(nose_, LV_OBJ_FLAG_HIDDEN);
  lv_obj_clear_flag(route_, LV_OBJ_FLAG_HIDDEN);

  // integrate the coded moves into a list of key poses. Drives that overshoot
  // into a wall (e.g. the elim "4 pins" wall-squaring moves) are clamped to the
  // field boundary -- the real robot pins against the wall there, so the encoder
  // target beyond it just stalls. This changes no auton value; it only stops the
  // drawn square at the wall instead of marching it off the field.
  const double B = FIELD_IN / 2.0 - (ROBOT_SZ / 2.0) / FIELD_PX * FIELD_IN;  // ~68"
  std::vector<Pose> poses;
  Pose cur = pv.start;
  poses.push_back(cur);
  for (const Move& m : pv.moves) {
    if (m.kind == 'D') {
      double r = cur.t * M_PI / 180.0;
      cur.x += m.v * std::sin(r);
      cur.y += m.v * std::cos(r);
      cur.x = cur.x < -B ? -B : (cur.x > B ? B : cur.x);  // pin at walls
      cur.y = cur.y < -B ? -B : (cur.y > B ? B : cur.y);
    } else if (m.kind == 'T') {
      cur.t += m.v;
    } else if (m.kind == 'A') {
      cur.t = m.v;
    }
    poses.push_back(cur);
  }
  if (pv.mirror) {
    for (Pose& q : poses) {  // blue = red flipped about field center
      q.x = -q.x;
      q.y = -q.y;
      q.t += 180;
    }
  }

  // keyframe points for the route line
  for (const Pose& q : poses) route_pts_.push_back(to_px(q));
  lv_line_set_points(route_, route_pts_.data(), route_pts_.size());

  // densify into evenly-spaced points the square steps through, carrying heading
  // so the nose can point where the robot faces. Drives translate; turns keep the
  // position fixed and rotate the heading in place so you SEE the turn happen.
  auto push = [&](lv_point_t p, float hdg) {
    anim_pts_.push_back(p);
    anim_hdg_.push_back(hdg);
  };
  for (size_t i = 0; i + 1 < poses.size(); i++) {
    lv_point_t a = to_px(poses[i]), b = to_px(poses[i + 1]);
    double dx = b.x - a.x, dy = b.y - a.y;
    double dist = std::sqrt(dx * dx + dy * dy);

    if (dist < 1.0) {  // in-place turn: spin the nose from old heading to new
      double h0 = poses[i].t, h1 = poses[i + 1].t;
      double d = std::fmod(h1 - h0 + 540.0, 360.0) - 180.0;  // shortest way
      int steps = 10;
      for (int s = 0; s < steps; s++) push(a, (float)(h0 + d * s / steps));
    } else {  // drive: move, heading constant
      int steps = (int)std::max(4.0, dist / 2.0);
      for (int s = 0; s < steps; s++) {
        double f = (double)s / steps;
        push({(lv_coord_t)(a.x + dx * f), (lv_coord_t)(a.y + dy * f)},
             (float)poses[i + 1].t);
      }
    }
  }
  if (!route_pts_.empty()) {
    for (int i = 0; i < 25; i++) push(route_pts_.back(), (float)poses.back().t);  // end pause
  }
}

void BrainScreen::apply_selection() {
  for (size_t i = 0; i < rows_.size(); i++) {
    bool sel = ((int)i == selected_index_);
    lv_obj_set_style_outline_width(rows_[i], sel ? 3 : 0, LV_PART_MAIN);
    lv_obj_set_style_bg_opa(rows_[i], sel ? LV_OPA_COVER : 150, LV_PART_MAIN);
    if (sel) {
      lv_obj_add_state(rows_[i], LV_STATE_CHECKED);
      lv_obj_scroll_to_view(rows_[i], LV_ANIM_ON);
    } else {
      lv_obj_clear_state(rows_[i], LV_STATE_CHECKED);
    }
  }
  rebuild_preview();
}

void BrainScreen::step_anim() {
  if (anim_pts_.empty()) return;
  const lv_point_t& p = anim_pts_[anim_i_];
  lv_obj_set_pos(robot_, p.x - ROBOT_SZ / 2, p.y - ROBOT_SZ / 2);

  // nose dot at the front of the square (shows heading -> turns are visible).
  // field "forward" for this heading is (-sin, -cos); screen y is flipped.
  double r = anim_hdg_[anim_i_] * M_PI / 180.0;
  const double NOSE = ROBOT_SZ * 0.55;
  int nx = (int)(p.x - NOSE * std::sin(r));
  int ny = (int)(p.y + NOSE * std::cos(r));
  lv_obj_set_pos(nose_, nx - 2, ny - 2);

  anim_i_ = (anim_i_ + 1) % anim_pts_.size();
}

// ===========================================================================
// selection API (no LVGL calls here -- just flag for the timer)
// ===========================================================================
void BrainScreen::select(int index) {
  if (autons_.empty()) return;
  if (index < 0) index = 0;
  if (index >= (int)autons_.size()) index = autons_.size() - 1;
  selected_index_ = index;
  text_mode_ = false;  // picking a new auton always goes back to the map view
  dirty_ = true;
}

void BrainScreen::show_text(const std::string& text) {
  pending_text_ = text;
  text_mode_ = true;
}

void BrainScreen::hide_text() {
  text_mode_ = false;
  dirty_ = true;
}

void BrainScreen::next() { select((selected_index_ + 1) % (int)autons_.size()); }
void BrainScreen::prev() {
  select((selected_index_ - 1 + (int)autons_.size()) % (int)autons_.size());
}

std::string BrainScreen::selected_name() const {
  if (autons_.empty()) return "";
  return autons_[selected_index_].name;
}

void BrainScreen::run_selected() {
  if (autons_.empty()) return;
  if (autons_[selected_index_].callback) autons_[selected_index_].callback();
}

// ---- static handlers ----
void BrainScreen::row_clicked(lv_event_t* e) {
  brain_screen.select((int)(intptr_t)lv_event_get_user_data(e));
}
void BrainScreen::scroll_up(lv_event_t* e) { brain_screen.prev(); }
void BrainScreen::scroll_down(lv_event_t* e) { brain_screen.next(); }

void BrainScreen::battery_timer(lv_timer_t* t) {
  if (!brain_screen.battery_) return;
  std::string s = std::to_string((int)pros::battery::get_capacity()) + "%";
  lv_label_set_text(brain_screen.battery_, s.c_str());
}

void BrainScreen::anim_timer(lv_timer_t* t) {
  if (brain_screen.text_mode_) {
    lv_label_set_text(brain_screen.map_text_, brain_screen.pending_text_.c_str());
    lv_obj_clear_flag(brain_screen.map_text_, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(brain_screen.field_, LV_OBJ_FLAG_HIDDEN);
    return;
  }
  lv_obj_add_flag(brain_screen.map_text_, LV_OBJ_FLAG_HIDDEN);
  lv_obj_clear_flag(brain_screen.field_, LV_OBJ_FLAG_HIDDEN);

  if (brain_screen.dirty_) {
    brain_screen.apply_selection();
    brain_screen.dirty_ = false;
  }
  brain_screen.step_anim();
}

// ===========================================================================
// auton list
// ===========================================================================
// no preview data yet for redLeft/redRight/blueLeft/blueRight -- they're still
// placeholder wiggle routines. Add a Move list here (see git history for the
// old QUALS_R/ELIM_R/YELLOWS_R style) once their real paths are written.
void brain_screen_init() {
  const lv_color_t RED = lv_color_hex(0xff3643);
  const lv_color_t BLUE = lv_color_hex(0x01b1f0);
  const lv_color_t GREEN = lv_color_hex(0x22d428);
  const lv_color_t GRAY = lv_color_hex(0x575757);

  std::vector<AutonItem> a = {
      {"redLeft", redLeft, "red, left side", RED, {}},
      {"redRight", redRight, "red, right side", RED, {}},
      {"blueLeft", blueLeft, "blue, left side", BLUE, {}},
      {"blueRight", blueRight, "blue, right side", BLUE, {}},
      {"skills", skills, "programming skills run", GREEN, {}},
      {"arm_height_capture", arm_height_capture, "jog arm, read live pos", GRAY, {}},
  };

  brain_screen.init(a);
}
