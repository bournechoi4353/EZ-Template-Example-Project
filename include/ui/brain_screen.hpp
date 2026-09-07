#pragma once

#include "api.h"                 // IWYU pragma: keep  (pulls in liblvgl + pros)
#include <functional>
#include <string>
#include <vector>

// ---- path preview model ----------------------------------------------------
// A robot pose on the field, in EZ odom units: inches from field center,
// heading in degrees (0 = +Y / up-field, positive = clockwise).
struct Pose {
  double x = 0, y = 0, t = 0;
};

// One coded motion, mirroring what the auton actually calls:
//   'D' = pid_drive_set(v inches)   (relative, signed: negative = reverse)
//   'T' = pid_turn_relative_set(v)  (relative turn, degrees)
//   'A' = pid_turn_set(v)           (absolute heading, degrees)
struct Move {
  char kind;
  double v;
};

// A selectable auton's preview: the coded moves, where it starts on the field,
// and whether to mirror it (blue alliance = red's path flipped about center).
struct Preview {
  bool has = false;
  Pose start;
  bool mirror = false;
  std::vector<Move> moves;
};

// ---- one selector entry ----------------------------------------------------
struct AutonItem {
  std::string name;
  std::function<void()> callback;
  std::string description;
  lv_color_t color;
  Preview preview;
};

// Touch auton selector built on LVGL. Three columns: the auton list (left),
// logo + scroll buttons (middle), and a field map with a square that animates
// along the selected auton's coded path (right). autonomous() runs whatever is
// selected here.
class BrainScreen {
 public:
  void init(const std::vector<AutonItem>& autons);
  void run_selected();

  void select(int index);
  void next();
  void prev();

  int selected_index() const { return selected_index_; }
  std::string selected_name() const;

  // shows text in place of the field-map panel -- for test autons (like
  // arm_height_capture) that need to print live values somewhere visible.
  // safe to call from any task; the actual LVGL update happens in the anim
  // timer. hide_text() (or selecting a different auton) restores the map.
  void show_text(const std::string& text);
  void hide_text();

 private:
  // Only these run LVGL calls, and only from the anim timer (LVGL's own
  // context) -- select()/next()/prev()/row clicks just flag dirty_.
  void apply_selection();                 // repaint checked row + rebuild preview
  void rebuild_preview();                 // integrate path -> screen points
  void step_anim();                       // advance the square one frame
  static void row_clicked(lv_event_t* e);
  static void scroll_up(lv_event_t* e);
  static void scroll_down(lv_event_t* e);
  static void battery_timer(lv_timer_t* t);
  static void anim_timer(lv_timer_t* t);

  // field-inch pose -> field-panel-local pixel
  lv_point_t to_px(const Pose& p) const;

  std::vector<AutonItem> autons_;
  int selected_index_ = 0;
  bool dirty_ = true;   // selection changed; timer must re-render

  // LVGL objects
  lv_obj_t* screen_ = nullptr;
  lv_obj_t* list_ = nullptr;
  lv_obj_t* battery_ = nullptr;
  lv_obj_t* field_ = nullptr;   // right panel (placeholder or field image)
  lv_obj_t* route_ = nullptr;   // faint polyline of the whole path
  lv_obj_t* robot_ = nullptr;   // the moving square
  lv_obj_t* nose_ = nullptr;    // heading dot on the square (shows turns)
  lv_obj_t* map_text_ = nullptr;  // covers the field panel to show show_text()
  std::vector<lv_obj_t*> rows_;

  // set from any task via show_text()/hide_text(); applied to map_text_ by
  // the anim timer.
  std::string pending_text_;
  bool text_mode_ = false;

  // animation state
  std::vector<lv_point_t> anim_pts_;   // dense screen points the square walks
  std::vector<float> anim_hdg_;        // heading (deg) at each dense point
  std::vector<lv_point_t> route_pts_;  // keyframe screen points for the line
  size_t anim_i_ = 0;
};

extern BrainScreen brain_screen;

void brain_screen_init();
