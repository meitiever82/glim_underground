#include <glim/viewer/editor/trajectory_qc_panel.hpp>

#include <sstream>
#include <iomanip>

#include <imgui.h>
#include <portable-file-dialogs.h>

#include <glk/thin_lines.hpp>
#include <glk/primitives/primitives.hpp>
#include <guik/viewer/light_viewer.hpp>

namespace glim {

namespace {
constexpr const char* DRAWABLE_TRAJ = "qc_trajectory";
constexpr const char* DRAWABLE_MARK = "qc_event_marker";

ImVec4 kind_color(const std::string& kind) {
  if (kind == "pose_jump") {
    return ImVec4(1.0f, 0.35f, 0.3f, 1.0f);
  } else if (kind == "data_gap") {
    return ImVec4(1.0f, 0.75f, 0.2f, 1.0f);
  }
  return ImVec4(0.6f, 0.8f, 1.0f, 1.0f);
}
}  // namespace

TrajectoryQCPanel::TrajectoryQCPanel(std::shared_ptr<spdlog::logger> logger) : logger(logger) {}

TrajectoryQCPanel::~TrajectoryQCPanel() {
  remove_drawables();
}

void TrajectoryQCPanel::clear() {
  submaps.clear();
  report = QCReport();
  selected_event = -1;
  status.clear();
  remove_drawables();
}

void TrajectoryQCPanel::set_submaps(const std::vector<SubMap::ConstPtr>& submaps_, const std::string& path) {
  submaps.assign(submaps_.begin(), submaps_.end());
  map_path = path;
  run_analysis();
}

void TrajectoryQCPanel::run_analysis() {
  if (submaps.empty()) {
    report = QCReport();
    status = "no map loaded";
    return;
  }

  report = TrajectoryQC::analyze(submaps, config);
  report.map_path = map_path;
  selected_event = -1;

  if (logger) {
    TrajectoryQC::log_report(report, logger);
  }
  status = report.verdict();
  update_drawables();
}

void TrajectoryQCPanel::remove_drawables() {
  auto viewer = guik::LightViewer::instance();
  if (!viewer) {
    return;
  }
  viewer->remove_drawable(DRAWABLE_TRAJ);
  viewer->remove_drawable(DRAWABLE_MARK);
}

void TrajectoryQCPanel::update_drawables() {
  auto viewer = guik::LightViewer::instance();
  if (!viewer) {
    return;
  }

  if (!show_overlay || !report.valid || report.stats.size() < 2) {
    remove_drawables();
    return;
  }

  // One line segment per consecutive frame pair, colored by severity.
  std::vector<Eigen::Vector3f> vertices;
  std::vector<Eigen::Vector4f> colors;
  vertices.reserve((report.stats.size() - 1) * 2);
  colors.reserve((report.stats.size() - 1) * 2);

  const Eigen::Vector4f col_ok(0.25f, 0.85f, 0.35f, 1.0f);
  const Eigen::Vector4f col_jump(1.0f, 0.15f, 0.1f, 1.0f);
  const Eigen::Vector4f col_fast(1.0f, 0.65f, 0.1f, 1.0f);

  for (size_t i = 1; i < report.stats.size(); i++) {
    const auto& a = report.stats[i - 1];
    const auto& b = report.stats[i];
    vertices.push_back(a.position.cast<float>());
    vertices.push_back(b.position.cast<float>());

    Eigen::Vector4f c = col_ok;
    if (b.flagged) {
      const bool jump = (b.v_pose > config.max_speed) || (b.accel > config.max_accel);
      c = jump ? col_jump : col_fast;
    }
    colors.push_back(c);
    colors.push_back(c);
  }

  auto lines = std::make_shared<glk::ThinLines>(vertices.data(), colors.data(), vertices.size(), false, 3.0f);
  viewer->update_drawable(DRAWABLE_TRAJ, lines, guik::VertexColor());

  if (selected_event >= 0 && selected_event < static_cast<int>(report.events.size())) {
    const auto& e = report.events[selected_event];
    const Eigen::Vector3f p = e.pos_begin.cast<float>();
    viewer->update_drawable(
      DRAWABLE_MARK,
      glk::Primitives::wire_sphere(),
      guik::FlatColor(1.0f, 1.0f, 0.2f, 1.0f, Eigen::Translation3f(p) * Eigen::Isometry3f::Identity() * Eigen::UniformScaling<float>(1.5f)));
  } else {
    viewer->remove_drawable(DRAWABLE_MARK);
  }
}

void TrajectoryQCPanel::focus_on(const QCEvent& event) {
  auto viewer = guik::LightViewer::instance();
  if (viewer) {
    viewer->lookat(event.pos_begin.cast<float>());
  }
}

void TrajectoryQCPanel::draw_ui() {
  if (request_open) {
    show_window = true;
    request_open = false;
    ImGui::SetNextWindowSize(ImVec2(620, 480), ImGuiCond_FirstUseEver);
  }
  if (!show_window) {
    return;
  }

  if (!ImGui::Begin("Trajectory QC", &show_window)) {
    ImGui::End();
    return;
  }

  if (!report.valid) {
    ImGui::TextDisabled("%s", status.empty() ? "no map loaded" : status.c_str());
    if (ImGui::Button("Run check") && !submaps.empty()) {
      run_analysis();
    }
    ImGui::End();
    return;
  }

  // ---- verdict ----
  const bool ok = report.passed();
  ImGui::PushStyleColor(ImGuiCol_Text, ok ? ImVec4(0.3f, 0.9f, 0.4f, 1.0f) : ImVec4(1.0f, 0.35f, 0.3f, 1.0f));
  ImGui::Text("%s", report.verdict().c_str());
  ImGui::PopStyleColor();
  ImGui::Separator();

  // ---- summary ----
  ImGui::Text("submaps %d   frames %d   duration %.1f s", report.n_submaps, report.n_frames, report.duration);
  ImGui::Text("path %.1f m   start-end offset %.2f m", report.path_length, report.straight_line);
  ImGui::Text("speed max %.2f m/s (p99 %.2f)   max frame step %.3f m", report.max_v_pose, report.p99_v_pose, report.max_frame_step);
  ImGui::TextDisabled("note: v_imu is an optimized state, not an independent measurement - it jumps with the pose");
  if (!report.imu_available) {
    ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 0.75f, 0.2f, 1.0f));
    ImGui::TextWrapped("IMU velocity state not available - judgement relies on kinematic limits only.");
    ImGui::PopStyleColor();
  }

  ImGui::Separator();

  // ---- thresholds ----
  if (ImGui::CollapsingHeader("Thresholds")) {
    bool dirty = false;
    dirty |= ImGui::DragScalar("max speed [m/s]", ImGuiDataType_Double, &config.max_speed, 0.01f);
    dirty |= ImGui::DragScalar("max accel [m/s2]", ImGuiDataType_Double, &config.max_accel, 0.1f);
    dirty |= ImGui::DragScalar("max IMU mismatch [m/s]", ImGuiDataType_Double, &config.max_imu_mismatch, 0.01f);
    dirty |= ImGui::DragScalar("max angular rate [deg/s]", ImGuiDataType_Double, &config.max_angular_rate, 1.0f);
    dirty |= ImGui::DragScalar("max frame gap [s]", ImGuiDataType_Double, &config.max_gap, 0.01f);
    dirty |= ImGui::DragScalar("merge gap [s]", ImGuiDataType_Double, &config.merge_gap, 0.05f);
    if (dirty) {
      run_analysis();
    }
    ImGui::SameLine();
    if (ImGui::Button("Reset")) {
      config = TrajectoryQCConfig();
      run_analysis();
    }
  }

  if (ImGui::Checkbox("Show overlay in 3D", &show_overlay)) {
    update_drawables();
  }
  ImGui::SameLine();
  ImGui::TextDisabled("(green ok / orange fast / red jump)");

  ImGui::Separator();

  // ---- events ----
  ImGui::Text("Events: %zu", report.events.size());
  if (ImGui::BeginTable("qc_events", 6, ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg | ImGuiTableFlags_ScrollY, ImVec2(0, 200))) {
    ImGui::TableSetupColumn("#", ImGuiTableColumnFlags_WidthFixed, 28.0f);
    ImGui::TableSetupColumn("kind", ImGuiTableColumnFlags_WidthFixed, 90.0f);
    ImGui::TableSetupColumn("t [s]", ImGuiTableColumnFlags_WidthFixed, 110.0f);
    ImGui::TableSetupColumn("travel", ImGuiTableColumnFlags_WidthFixed, 70.0f);
    ImGui::TableSetupColumn("peak v", ImGuiTableColumnFlags_WidthFixed, 80.0f);
    ImGui::TableSetupColumn("IMU v", ImGuiTableColumnFlags_WidthFixed, 70.0f);
    ImGui::TableHeadersRow();

    for (size_t i = 0; i < report.events.size(); i++) {
      const auto& e = report.events[i];
      ImGui::TableNextRow();
      ImGui::TableNextColumn();

      char label[32];
      snprintf(label, sizeof(label), "%zu", i);
      if (ImGui::Selectable(label, selected_event == static_cast<int>(i), ImGuiSelectableFlags_SpanAllColumns)) {
        selected_event = static_cast<int>(i);
        focus_on(e);
        update_drawables();
      }

      ImGui::TableNextColumn();
      ImGui::TextColored(kind_color(e.kind), "%s", e.kind.c_str());
      ImGui::TableNextColumn();
      ImGui::Text("%.1f - %.1f", e.t_begin, e.t_end);
      ImGui::TableNextColumn();
      ImGui::Text("%.2f m", e.displacement);
      ImGui::TableNextColumn();
      ImGui::Text("%.2f m/s", e.peak_v_pose);
      ImGui::TableNextColumn();
      if (report.imu_available) {
        ImGui::Text("%.2f", e.peak_v_imu);
      } else {
        ImGui::TextDisabled("n/a");
      }
    }
    ImGui::EndTable();
  }

  if (selected_event >= 0 && selected_event < static_cast<int>(report.events.size())) {
    const auto& e = report.events[selected_event];
    ImGui::TextWrapped("selected: %s", e.summary().c_str());
    if (ImGui::Button("Jump to event")) {
      focus_on(e);
    }
    ImGui::SameLine();
  }

  if (ImGui::Button("Export report")) {
    const std::string path = pfd::save_file("Save QC report", "trajectory_qc.txt").result();
    if (!path.empty()) {
      if (TrajectoryQC::write_report(report, path)) {
        status = "saved " + path;
        if (logger) {
          logger->info("trajectory QC report saved to {}", path);
        }
      } else {
        status = "failed to save " + path;
      }
    }
  }
  ImGui::SameLine();
  if (ImGui::Button("Re-run")) {
    run_analysis();
  }

  if (!status.empty()) {
    ImGui::TextDisabled("%s", status.c_str());
  }

  ImGui::End();
}

}  // namespace glim
