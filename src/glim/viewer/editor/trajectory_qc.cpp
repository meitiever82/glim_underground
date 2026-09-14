#include <glim/viewer/editor/trajectory_qc.hpp>

#include <cmath>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <algorithm>

namespace glim {

namespace {

/// @brief Flatten submaps into a world-frame trajectory.
///
/// Frame poses inside a submap live in that submap's odometry frame, so each one
/// has to be lifted into the world through the submap's left endpoint:
///   T_world_imu = (T_world_origin * T_origin_endpoint_L) * T_odom_imu0^-1 * frame->T_world_imu
/// The IMU velocity is a world-frame vector in that same odometry frame, so only
/// its rotation part is applied.
struct WorldFrame {
  double stamp;
  Eigen::Vector3d position;
  Eigen::Matrix3d rotation;
  Eigen::Vector3d velocity;
  bool has_velocity;
};

std::vector<WorldFrame> flatten(const std::vector<SubMap::ConstPtr>& submaps) {
  std::vector<WorldFrame> out;
  for (const auto& submap : submaps) {
    if (!submap || submap->frames.empty()) {
      continue;
    }

    const Eigen::Isometry3d T_world_endpoint_L = submap->T_world_origin * submap->T_origin_endpoint_L;
    const Eigen::Isometry3d T_odom_imu0 = submap->frames.front()->T_world_imu;
    const Eigen::Isometry3d T_world_odom = T_world_endpoint_L * T_odom_imu0.inverse();

    for (const auto& frame : submap->frames) {
      if (!frame) {
        continue;
      }
      const Eigen::Isometry3d T_world_imu = T_world_odom * frame->T_world_imu;

      WorldFrame wf;
      wf.stamp = frame->stamp;
      wf.position = T_world_imu.translation();
      wf.rotation = T_world_imu.linear();
      wf.velocity = T_world_odom.linear() * frame->v_world_imu;
      wf.has_velocity = frame->v_world_imu.allFinite() && frame->v_world_imu.norm() > 1e-9;
      out.push_back(wf);
    }
  }

  std::sort(out.begin(), out.end(), [](const WorldFrame& a, const WorldFrame& b) { return a.stamp < b.stamp; });

  // Drop duplicated stamps at submap boundaries (submaps overlap by design).
  out.erase(
    std::unique(out.begin(), out.end(), [](const WorldFrame& a, const WorldFrame& b) { return std::abs(a.stamp - b.stamp) < 1e-6; }),
    out.end());
  return out;
}

double percentile(std::vector<double> v, double p) {
  if (v.empty()) {
    return 0.0;
  }
  const size_t k = std::min(v.size() - 1, static_cast<size_t>(p / 100.0 * (v.size() - 1)));
  std::nth_element(v.begin(), v.begin() + k, v.end());
  return v[k];
}

}  // namespace

std::string QCEvent::summary() const {
  std::ostringstream ss;
  ss << std::fixed << std::setprecision(1) << "t=" << t_begin << "~" << t_end << "s  " << n_frames << " frames, " << std::setprecision(2) << displacement
     << " m, peak " << peak_v_pose << " m/s";
  if (kind == "pose_jump") {
    ss << ", peak accel " << peak_accel << " m/s2";
  }
  return ss.str();
}

bool QCReport::passed() const {
  if (!valid) {
    return false;
  }
  for (const auto& e : events) {
    if (e.kind == "pose_jump" || e.kind == "data_gap" || e.kind == "attitude_jump") {
      return false;
    }
  }
  return true;
}

std::string QCReport::verdict() const {
  if (!valid) {
    return "NO DATA";
  }
  int jumps = 0, gaps = 0, att = 0, fast = 0;
  for (const auto& e : events) {
    if (e.kind == "pose_jump") {
      jumps++;
    } else if (e.kind == "data_gap") {
      gaps++;
    } else if (e.kind == "attitude_jump") {
      att++;
    } else {
      fast++;
    }
  }

  std::ostringstream ss;
  if (jumps == 0 && gaps == 0 && att == 0) {
    ss << "PASS";
    if (fast) {
      ss << " (" << fast << " minor anomaly(ies), within kinematic limits)";
    }
  } else {
    ss << "FAIL - ";
    bool first = true;
    if (jumps) {
      ss << jumps << " pose jump(s) totalling " << std::fixed << std::setprecision(1) << flagged_displacement << " m";
      first = false;
    }
    if (att) {
      ss << (first ? "" : ", ") << att << " attitude jump(s)";
      first = false;
    }
    if (gaps) {
      ss << (first ? "" : ", ") << gaps << " data gap(s)";
    }
  }
  return ss.str();
}

QCReport TrajectoryQC::analyze(const std::vector<SubMap::Ptr>& submaps, const TrajectoryQCConfig& config) {
  std::vector<SubMap::ConstPtr> c(submaps.begin(), submaps.end());
  return analyze(c, config);
}

QCReport TrajectoryQC::analyze(const std::vector<SubMap::ConstPtr>& submaps, const TrajectoryQCConfig& config) {
  QCReport report;
  report.n_submaps = submaps.size();

  const auto frames = flatten(submaps);
  if (frames.size() < 2) {
    return report;
  }

  report.valid = true;
  report.n_frames = frames.size();
  report.duration = frames.back().stamp - frames.front().stamp;

  const double t0 = frames.front().stamp;
  std::vector<double> speeds;
  speeds.reserve(frames.size());

  report.stats.resize(frames.size());
  report.stats[0].stamp = 0.0;
  report.stats[0].position = frames[0].position;
  report.stats[0].dt = 0.0;
  report.stats[0].v_pose = 0.0;
  report.stats[0].v_imu = frames[0].velocity.norm();
  report.stats[0].imu_mismatch = 0.0;
  report.stats[0].angular_rate = 0.0;
  report.stats[0].has_imu = frames[0].has_velocity;
  report.stats[0].flagged = false;

  for (size_t i = 1; i < frames.size(); i++) {
    const auto& prev = frames[i - 1];
    const auto& curr = frames[i];

    const double dt = curr.stamp - prev.stamp;
    const double step = (curr.position - prev.position).norm();
    const double v_pose = dt > 1e-6 ? step / dt : 0.0;
    const double v_imu = curr.velocity.norm();

    // Relative rotation magnitude between consecutive frames
    const Eigen::Matrix3d dR = prev.rotation.transpose() * curr.rotation;
    const double cos_theta = std::min(1.0, std::max(-1.0, (dR.trace() - 1.0) * 0.5));
    const double omega = dt > 1e-6 ? std::acos(cos_theta) * 180.0 / M_PI / dt : 0.0;

    const double prev_v = report.stats[i - 1].v_pose;
    const double accel = dt > 1e-6 ? std::abs(v_pose - prev_v) / dt : 0.0;

    auto& s = report.stats[i];
    s.stamp = curr.stamp - t0;
    s.position = curr.position;
    s.dt = dt;
    s.v_pose = v_pose;
    s.v_imu = v_imu;
    s.has_imu = curr.has_velocity;
    s.imu_mismatch = curr.has_velocity ? std::abs(v_pose - v_imu) : 0.0;
    s.angular_rate = omega;
    s.accel = accel;
    s.flagged = (v_pose > config.max_speed) ||                                              //
                (accel > config.max_accel) ||                                               //
                (curr.has_velocity && s.imu_mismatch > config.max_imu_mismatch) ||          //
                (omega > config.max_angular_rate) ||                                        //
                (dt > config.max_gap);

    report.path_length += step;
    report.max_frame_step = std::max(report.max_frame_step, step);
    report.max_v_pose = std::max(report.max_v_pose, v_pose);
    if (curr.has_velocity) {
      report.imu_available = true;
    }
    if (s.flagged) {
      report.n_flagged++;
    }
    speeds.push_back(v_pose);
  }

  report.p99_v_pose = percentile(speeds, 99.0);
  report.straight_line = (frames.back().position - frames.front().position).norm();

  // Group flagged frames into events (gaps shorter than merge_gap stay in one event)
  for (size_t i = 1; i < report.stats.size();) {
    if (!report.stats[i].flagged) {
      i++;
      continue;
    }

    size_t j = i;
    size_t last_flagged = i;
    while (j + 1 < report.stats.size() && report.stats[j + 1].stamp - report.stats[last_flagged].stamp < config.merge_gap) {
      j++;
      if (report.stats[j].flagged) {
        last_flagged = j;
      }
    }
    j = last_flagged;

    QCEvent ev;
    ev.t_begin = report.stats[i].stamp;
    ev.t_end = report.stats[j].stamp;
    ev.n_frames = static_cast<int>(j - i + 1);
    ev.pos_begin = report.stats[i].position;
    ev.pos_end = report.stats[j].position;
    ev.displacement = 0.0;
    ev.peak_v_pose = 0.0;
    ev.peak_v_imu = 0.0;
    ev.peak_mismatch = 0.0;
    ev.peak_angular_rate = 0.0;
    ev.peak_accel = 0.0;

    bool any_gap = false;
    bool imu_seen = false;
    for (size_t k = i; k <= j; k++) {
      const auto& s = report.stats[k];
      ev.displacement += s.v_pose * s.dt;
      ev.peak_v_pose = std::max(ev.peak_v_pose, s.v_pose);
      ev.peak_v_imu = std::max(ev.peak_v_imu, s.v_imu);
      ev.peak_mismatch = std::max(ev.peak_mismatch, s.imu_mismatch);
      ev.peak_angular_rate = std::max(ev.peak_angular_rate, s.angular_rate);
      ev.peak_accel = std::max(ev.peak_accel, s.accel);
      any_gap = any_gap || (s.dt > config.max_gap);
      imu_seen = imu_seen || s.has_imu;
    }

    // Classify. Kinematic violation is the primary evidence: the velocity state is
    // optimized jointly with the pose, so it jumps along with it and cannot exonerate
    // a jump. IMU mismatch only catches the rarer pose/velocity disagreement.
    (void)imu_seen;
    if (any_gap) {
      ev.kind = "data_gap";
    } else if (ev.peak_v_pose > config.max_speed || ev.peak_accel > config.max_accel) {
      ev.kind = "pose_jump";
    } else if (ev.peak_mismatch > config.max_imu_mismatch) {
      ev.kind = "pose_jump";
    } else if (ev.peak_angular_rate > config.max_angular_rate) {
      ev.kind = "attitude_jump";
    } else {
      ev.kind = "fast_motion";
    }

    if (ev.n_frames >= config.min_event_frames) {
      if (ev.kind != "fast_motion") {
        report.flagged_displacement += ev.displacement;
      }
      report.events.push_back(ev);
    }
    i = j + 1;
  }

  return report;
}

bool TrajectoryQC::write_report(const QCReport& report, const std::string& path) {
  std::ofstream ofs(path);
  if (!ofs) {
    return false;
  }

  ofs << std::fixed << std::setprecision(3);
  ofs << "# GLIM trajectory quality check\n";
  ofs << "map              : " << report.map_path << "\n";
  ofs << "verdict          : " << report.verdict() << "\n\n";
  ofs << "submaps          : " << report.n_submaps << "\n";
  ofs << "frames           : " << report.n_frames << "\n";
  ofs << "duration         : " << report.duration << " s\n";
  ofs << "path length      : " << report.path_length << " m\n";
  ofs << "start-end offset : " << report.straight_line << " m\n";
  ofs << "max speed        : " << report.max_v_pose << " m/s\n";
  ofs << "p99 speed        : " << report.p99_v_pose << " m/s\n";
  ofs << "max frame step   : " << report.max_frame_step << " m\n";
  ofs << "IMU velocity     : " << (report.imu_available ? "available" : "NOT available") << "\n";
  ofs << "flagged frames   : " << report.n_flagged << "\n";
  ofs << "suspect travel   : " << report.flagged_displacement << " m\n\n";

  ofs << "events (" << report.events.size() << "):\n";
  for (size_t i = 0; i < report.events.size(); i++) {
    const auto& e = report.events[i];
    ofs << "  [" << i << "] " << std::setw(12) << std::left << e.kind << " " << e.summary() << "\n";
    ofs << "        from (" << e.pos_begin.x() << ", " << e.pos_begin.y() << ", " << e.pos_begin.z() << ")"
        << " to (" << e.pos_end.x() << ", " << e.pos_end.y() << ", " << e.pos_end.z() << ")\n";
  }

  ofs.close();

  std::ofstream csv(path + ".csv");
  if (csv) {
    csv << "t,x,y,z,dt,v_pose,v_imu,imu_mismatch,accel,angular_rate,flagged\n";
    csv << std::fixed << std::setprecision(4);
    for (const auto& s : report.stats) {
      csv << s.stamp << "," << s.position.x() << "," << s.position.y() << "," << s.position.z() << "," << s.dt << "," << s.v_pose << "," << s.v_imu << ","
          << s.imu_mismatch << "," << s.accel << "," << s.angular_rate << "," << (s.flagged ? 1 : 0) << "\n";
    }
  }
  return true;
}

void TrajectoryQC::log_report(const QCReport& report, std::shared_ptr<spdlog::logger> logger) {
  if (!logger) {
    logger = spdlog::default_logger();
  }
  if (!report.valid) {
    logger->error("trajectory QC: no usable frames");
    return;
  }

  logger->info("trajectory QC: {}", report.verdict());
  logger->info(
    "  {} submaps / {} frames, {:.1f} s, path {:.1f} m, start-end {:.2f} m",
    report.n_submaps,
    report.n_frames,
    report.duration,
    report.path_length,
    report.straight_line);
  logger->info(
    "  speed max {:.2f} m/s (p99 {:.2f}), max frame step {:.3f} m, IMU velocity {}",
    report.max_v_pose,
    report.p99_v_pose,
    report.max_frame_step,
    report.imu_available ? "available" : "NOT available");

  for (size_t i = 0; i < report.events.size(); i++) {
    const auto& e = report.events[i];
    if (e.kind == "pose_jump") {
      logger->error("  [{}] {:<12} {}", i, e.kind, e.summary());
    } else if (e.kind == "data_gap") {
      logger->warn("  [{}] {:<12} {}", i, e.kind, e.summary());
    } else {
      logger->info("  [{}] {:<12} {}", i, e.kind, e.summary());
    }
  }
}

}  // namespace glim
