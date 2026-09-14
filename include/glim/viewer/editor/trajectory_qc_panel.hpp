#pragma once

#include <memory>
#include <string>
#include <vector>
#include <spdlog/spdlog.h>

#include <glim/mapping/sub_map.hpp>
#include <glim/viewer/editor/trajectory_qc.hpp>

namespace glim {

/// @brief Interactive trajectory quality check panel for the offline viewer.
///
/// Highlights anomalous trajectory segments in 3D (red = pose jump, orange = fast
/// motion) and lists the events so they can be inspected one by one.
class TrajectoryQCPanel {
public:
  TrajectoryQCPanel(std::shared_ptr<spdlog::logger> logger);
  ~TrajectoryQCPanel();

  /// @brief Feed the loaded map. Runs the check immediately.
  void set_submaps(const std::vector<SubMap::ConstPtr>& submaps, const std::string& map_path = "");
  void clear();

  void show() { request_open = true; }
  bool has_result() const { return report.valid; }
  const QCReport& result() const { return report; }

  /// @brief Draw the panel. Call once per frame from a UI callback.
  void draw_ui();

private:
  void run_analysis();
  void update_drawables();
  void remove_drawables();
  void focus_on(const QCEvent& event);

private:
  std::shared_ptr<spdlog::logger> logger;

  std::vector<SubMap::ConstPtr> submaps;
  std::string map_path;

  TrajectoryQCConfig config;
  QCReport report;

  bool request_open = false;
  bool show_window = false;
  bool show_overlay = true;   ///< draw the colored trajectory in 3D
  int selected_event = -1;
  std::string status;
};

}  // namespace glim
