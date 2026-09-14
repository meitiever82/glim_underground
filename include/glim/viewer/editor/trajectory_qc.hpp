#pragma once

#include <string>
#include <vector>
#include <memory>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <spdlog/spdlog.h>

#include <glim/mapping/sub_map.hpp>

namespace glim {

/// @brief Thresholds for trajectory quality check
struct TrajectoryQCConfig {
  double max_speed = 1.5;          ///< [m/s] Above the platform's kinematic limit -> physically impossible
  double max_accel = 6.0;          ///< [m/s^2] Above the platform's kinematic limit
  double max_imu_mismatch = 0.8;   ///< [m/s] |v_pose - v_imu|; secondary signal only (see note below)
  double max_angular_rate = 90.0;  ///< [deg/s] Angular rate above this is suspicious
  double max_gap = 0.5;            ///< [s] Frame interval above this is a data gap
  double merge_gap = 2.0;          ///< [s] Anomalies closer than this are merged into one event
  double min_event_frames = 2;     ///< Events with fewer frames are ignored
};

/// @brief Per-frame statistics
struct QCFrameStat {
  double stamp;
  Eigen::Vector3d position;
  double dt;             ///< [s] interval from the previous frame
  double v_pose;         ///< [m/s] speed from pose differentiation (what SLAM thinks it moved)
  double v_imu;          ///< [m/s] speed from IMU state (independent motion estimate)
  double imu_mismatch;   ///< [m/s] |v_pose - v_imu|
  double angular_rate;   ///< [deg/s]
  double accel;          ///< [m/s^2] magnitude of speed change
  bool has_imu;          ///< IMU velocity available
  bool flagged;          ///< violates any threshold
};

/// @brief A contiguous run of anomalous frames
struct QCEvent {
  double t_begin;        ///< [s] relative to trajectory start
  double t_end;
  int n_frames;
  double displacement;   ///< [m] accumulated travel during the event
  double peak_v_pose;
  double peak_v_imu;
  double peak_mismatch;
  double peak_angular_rate;
  double peak_accel;
  Eigen::Vector3d pos_begin;
  Eigen::Vector3d pos_end;

  /// @brief "pose_jump":     motion beyond the platform's kinematic limits -> localization failure.
  ///        "attitude_jump": angular rate beyond limits.
  ///        "data_gap":      frame interval too long.
  ///        "fast_motion":   flagged but within kinematic limits (e.g. mild IMU mismatch).
  std::string kind;

  std::string summary() const;
};

/// @brief Result of a trajectory quality check
struct QCReport {
  bool valid = false;
  std::string map_path;

  std::vector<QCFrameStat> stats;
  std::vector<QCEvent> events;

  int n_submaps = 0;
  int n_frames = 0;
  double duration = 0.0;         ///< [s]
  double path_length = 0.0;      ///< [m]
  double straight_line = 0.0;    ///< [m] start-to-end displacement
  double max_v_pose = 0.0;
  double p99_v_pose = 0.0;
  double max_frame_step = 0.0;   ///< [m] largest single-frame displacement
  int n_flagged = 0;
  double flagged_displacement = 0.0;  ///< [m] travel accumulated inside anomalous events
  bool imu_available = false;

  /// @brief No pose jumps and no data gaps.
  bool passed() const;
  /// @brief One-line verdict for logs / CLI.
  std::string verdict() const;
};

/// @brief Trajectory quality check.
///
/// Detects SLAM failures that are invisible to per-frame residuals. Feature-poor
/// geometry (long straight culverts, tunnels, corridors) makes LiDAR odometry slide
/// along the degenerate axis; the slide shows up here as a burst of frames whose
/// pose-differentiated speed or acceleration exceeds what the platform can do.
///
/// @note `EstimationFrame::v_world_imu` is an *optimized state*, not an independent
///       IMU measurement. When the pose jumps, the velocity state jumps with it and
///       stays self-consistent - measured on a real 39 m jump, |v_pose - v_imu| was
///       0.27 m/s versus 0.25 m/s on normal motion. So the IMU mismatch cannot veto a
///       kinematic violation; it is kept only as a secondary signal for the rarer case
///       where pose and velocity states disagree. The primary evidence is kinematics:
///       a legged robot cannot travel at 4 m/s.
class TrajectoryQC {
public:
  /// @brief Run the check over loaded submaps.
  static QCReport analyze(const std::vector<SubMap::ConstPtr>& submaps, const TrajectoryQCConfig& config = TrajectoryQCConfig());

  /// @brief Convenience overload for non-const submaps.
  static QCReport analyze(const std::vector<SubMap::Ptr>& submaps, const TrajectoryQCConfig& config = TrajectoryQCConfig());

  /// @brief Write a human-readable report. Also writes <path>.csv with per-frame stats.
  static bool write_report(const QCReport& report, const std::string& path);

  /// @brief Print the report to a logger.
  static void log_report(const QCReport& report, std::shared_ptr<spdlog::logger> logger);
};

}  // namespace glim
