// ROS 2 node wrapping PGOBackend.
//
// Subscribes
//   odom_topic   nav_msgs/Odometry        FAST-LIO2 (livox_frame w.r.t. its
//                                         start pose, i.e. the odom frame)
//   cloud_topic  sensor_msgs/PointCloud2  deskewed scan, synced with odometry
//   navpvt_topic ublox_msgs/NavPVT        RTK carrier solution + position
//   gps_topic    sensor_msgs/NavSatFix    fallback if gps_input_mode=navsatfix
//
// Publishes
//   pgo/odometry  corrected base_link pose at full LIO rate
//   pgo/pose      optimized pose at keyframe rate
//   pgo/path      optimized keyframe trajectory
//   pgo/map       aggregated map (on save / periodic)
//   pgo/loop_markers
//   TF map -> odom (odom frame id taken from the Odometry messages)
//
// Service:  ~/save (std_srvs/Trigger)
//
// Lever arms are read from TF (static): livox_frame -> gps_frame for the
// antenna, livox_frame -> base_link for output poses. Falls back to the
// antenna_lever / t_body_base parameters if TF is unavailable.
#include <atomic>
#include <cstdlib>
#include <memory>
#include <mutex>
#include <thread>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <ublox_msgs/msg/nav_pvt.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include "lidar_gps_pgo/pgo_backend.hpp"

namespace lidar_gps_pgo {

namespace {

gtsam::Pose3 fromMsg(const geometry_msgs::msg::Pose& p) {
  return gtsam::Pose3(
      gtsam::Rot3::Quaternion(p.orientation.w, p.orientation.x,
                              p.orientation.y, p.orientation.z),
      gtsam::Point3(p.position.x, p.position.y, p.position.z));
}

void toMsg(const gtsam::Pose3& pose, geometry_msgs::msg::Pose& out) {
  const auto t = pose.translation();
  const auto q = pose.rotation().toQuaternion();
  out.position.x = t.x();
  out.position.y = t.y();
  out.position.z = t.z();
  out.orientation.w = q.w();
  out.orientation.x = q.x();
  out.orientation.y = q.y();
  out.orientation.z = q.z();
}

double toSec(const builtin_interfaces::msg::Time& t) {
  return t.sec + 1e-9 * t.nanosec;
}

}  // namespace

class PGONode : public rclcpp::Node {
 public:
  PGONode() : Node("pgo") {
    declareParams();
    BackendConfig cfg = readConfig();
    backend_ = std::make_unique<PGOBackend>(
        cfg, [this](const std::string& m) {
          RCLCPP_INFO(get_logger(), "%s", m.c_str());
        });

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    const auto qos = rclcpp::SensorDataQoS().keep_last(50);
    // The odom/cloud pair must be RELIABLE: FAST-LIO publishes /cloud_registered
    // reliably, and a best-effort reader silently drops the large multi-fragment
    // PointCloud2 samples (small Odometry still gets through), so the
    // ApproximateTime sync would never fire and no keyframes would form.
    const auto sync_qos = rclcpp::QoS(rclcpp::KeepLast(50)).reliable();
    sub_odom_.subscribe(this, get_parameter("odom_topic").as_string(),
                        sync_qos.get_rmw_qos_profile());
    sub_cloud_.subscribe(this, get_parameter("cloud_topic").as_string(),
                         sync_qos.get_rmw_qos_profile());
    sync_ = std::make_unique<message_filters::Synchronizer<SyncPolicy>>(
        SyncPolicy(100), sub_odom_, sub_cloud_);
    sync_->setMaxIntervalDuration(rclcpp::Duration::from_seconds(
        get_parameter("sync_slop").as_double()));
    sync_->registerCallback(&PGONode::cbSynced, this);

    gps_input_mode_ = get_parameter("gps_input_mode").as_string();
    if (gps_input_mode_ == "navpvt") {
      // position + quality both from NavPVT
      sub_navpvt_ = create_subscription<ublox_msgs::msg::NavPVT>(
          get_parameter("navpvt_topic").as_string(), qos,
          std::bind(&PGONode::cbNavPvt, this, std::placeholders::_1));
    } else if (gps_input_mode_ == "fused") {
      // position + covariance from NavSatFix; RTK quality from the paired
      // NavPVT carrier-solution flag (used to scale the covariance).
      sub_navpvt_ = create_subscription<ublox_msgs::msg::NavPVT>(
          get_parameter("navpvt_topic").as_string(), qos,
          std::bind(&PGONode::cbNavPvtFlag, this, std::placeholders::_1));
      sub_fix_ = create_subscription<sensor_msgs::msg::NavSatFix>(
          get_parameter("gps_topic").as_string(), qos,
          std::bind(&PGONode::cbNavSatFix, this, std::placeholders::_1));
    } else {  // navsatfix: classify by covariance thresholds
      sub_fix_ = create_subscription<sensor_msgs::msg::NavSatFix>(
          get_parameter("gps_topic").as_string(), qos,
          std::bind(&PGONode::cbNavSatFix, this, std::placeholders::_1));
    }

    pub_odom_ = create_publisher<nav_msgs::msg::Odometry>("pgo/odometry", 10);
    pub_pose_ = create_publisher<nav_msgs::msg::Odometry>("pgo/pose", 10);
    pub_path_ = create_publisher<nav_msgs::msg::Path>("pgo/path", 2);
    pub_map_ = create_publisher<sensor_msgs::msg::PointCloud2>("pgo/map", 1);
    pub_markers_ =
        create_publisher<visualization_msgs::msg::Marker>("pgo/loop_markers", 2);
    srv_save_ = create_service<std_srvs::srv::Trigger>(
        "~/save", std::bind(&PGONode::cbSave, this, std::placeholders::_1,
                            std::placeholders::_2));

    loop_thread_ = std::thread([this] { loopWorker(); });

    const double map_period = get_parameter("map_pub_period").as_double();
    if (map_period > 0.0) {
      map_timer_ = create_wall_timer(
          std::chrono::duration<double>(map_period),
          [this] { publishMap(); });
    }
    RCLCPP_INFO(get_logger(), "pgo node up (gps_input_mode=%s)",
                gps_input_mode_.c_str());
  }

  ~PGONode() override {
    running_ = false;
    if (loop_thread_.joinable()) loop_thread_.join();
  }

 private:
  using SyncPolicy = message_filters::sync_policies::ApproximateTime<
      nav_msgs::msg::Odometry, sensor_msgs::msg::PointCloud2>;

  // ------------------------------------------------------------- parameters
  void declareParams() {
    // topics / frames / node behaviour
    declare_parameter("odom_topic", std::string("/Odometry"));
    declare_parameter("cloud_topic", std::string("/cloud_registered"));
    declare_parameter("cloud_frame_mode", std::string("odom"));  // odom|body
    declare_parameter("gps_input_mode", std::string("navpvt"));
    declare_parameter("navpvt_topic", std::string("/navpvt"));
    declare_parameter("gps_topic", std::string("/fix"));
    declare_parameter("map_frame", std::string("map"));
    declare_parameter("base_frame", std::string("base_link"));
    declare_parameter("gps_frame", std::string("gps_frame"));
    declare_parameter("publish_tf", true);
    declare_parameter("publish_enu", true);   // publish map->odom/odometry in ENU
    declare_parameter("sync_slop", 0.1);
    declare_parameter("loop_period", 0.5);
    declare_parameter("map_pub_period", 0.0);
    declare_parameter("save_directory", std::string("~/pgo_output"));
    // fallback extrinsics if TF is unavailable (body = odometry child frame,
    // i.e. livox_frame for FAST-LIO2)
    declare_parameter("antenna_lever", std::vector<double>{0.0, 0.0, 0.0});
    declare_parameter("t_body_base", std::vector<double>{0.0, 0.0, 0.0});
    declare_parameter("q_body_base",
                      std::vector<double>{1.0, 0.0, 0.0, 0.0});  // w x y z
    // NavSatFix fallback classification thresholds
    declare_parameter("fixed_max_std", 0.05);
    declare_parameter("float_max_std", 1.0);

    // backend
    declare_parameter("keyframe_dist", 1.0);
    declare_parameter("keyframe_angle_deg", 15.0);
    declare_parameter("keyframe_voxel", 0.25);
    declare_parameter("cloud_min_range", 0.5);
    declare_parameter("cloud_max_range", 100.0);
    declare_parameter("prior_sigma_rot", 0.05);
    declare_parameter("prior_sigma_trans", 0.10);
    declare_parameter("odom_sigma_rot", 0.01);
    declare_parameter("odom_sigma_trans", 0.05);
    declare_parameter("use_gps", true);
    declare_parameter("use_float", true);
    declare_parameter("use_nonrtk", false);
    declare_parameter("fixed_sigma_floor", 0.02);
    declare_parameter("fixed_sigma_cap", 0.30);
    declare_parameter("float_sigma_floor", 0.30);
    declare_parameter("gps_cov_scale_fixed", 1.0);
    declare_parameter("gps_cov_scale_float", 25.0);
    declare_parameter("float_meas_sigma", 0.15);
    declare_parameter("float_bias_prior_sigma", 2.0);
    declare_parameter("float_bias_rw_sigma", 0.10);
    declare_parameter("float_bias_session_gap", 30.0);
    declare_parameter("nonrtk_sigma", 3.0);
    declare_parameter("gps_z_sigma_scale", 2.0);
    declare_parameter("gps_robust", true);
    declare_parameter("gps_time_tol", 0.20);
    declare_parameter("gps_max_interp_gap", 0.6);
    declare_parameter("gps_assoc_timeout", 5.0);
    declare_parameter("align_min_travel", 10.0);
    declare_parameter("align_min_travel_float", 40.0);
    declare_parameter("align_min_pairs", 10);
    declare_parameter("align_max_rms", 1.0);
    declare_parameter("align_max_rms_float", 2.5);
    declare_parameter("enable_loop_closure", true);
    declare_parameter("sc_num_ring", 20);
    declare_parameter("sc_num_sector", 60);
    declare_parameter("sc_max_radius", 80.0);
    declare_parameter("sc_dist_threshold", 0.13);
    declare_parameter("sc_num_candidates", 10);
    declare_parameter("sc_exclude_recent", 50);
    declare_parameter("sc_lidar_height", 1.0);
    declare_parameter("loop_min_query_gap", 5);
    declare_parameter("loop_max_per_tick", 2);
    declare_parameter("icp_method", std::string("gicp"));
    declare_parameter("icp_voxel", 0.4);
    declare_parameter("submap_size", 12);
    declare_parameter("icp_max_corr", 2.0);
    declare_parameter("icp_min_fitness", 0.45);
    declare_parameter("icp_max_rmse", 0.40);
    declare_parameter("loop_sigma_rot", 0.05);
    declare_parameter("loop_sigma_trans", 0.20);
    declare_parameter("loop_robust_k", 1.0);
    declare_parameter("map_voxel", 0.30);
  }

  BackendConfig readConfig() {
    BackendConfig c;
    auto d = [this](const char* n) { return get_parameter(n).as_double(); };
    auto i = [this](const char* n) {
      return static_cast<int>(get_parameter(n).as_int());
    };
    auto b = [this](const char* n) { return get_parameter(n).as_bool(); };
    c.keyframe_dist = d("keyframe_dist");
    c.keyframe_angle_deg = d("keyframe_angle_deg");
    c.keyframe_voxel = d("keyframe_voxel");
    c.cloud_min_range = d("cloud_min_range");
    c.cloud_max_range = d("cloud_max_range");
    c.prior_sigma_rot = d("prior_sigma_rot");
    c.prior_sigma_trans = d("prior_sigma_trans");
    c.odom_sigma_rot = d("odom_sigma_rot");
    c.odom_sigma_trans = d("odom_sigma_trans");
    c.use_gps = b("use_gps");
    c.use_float = b("use_float");
    c.use_nonrtk = b("use_nonrtk");
    c.fixed_sigma_floor = d("fixed_sigma_floor");
    c.fixed_sigma_cap = d("fixed_sigma_cap");
    c.float_sigma_floor = d("float_sigma_floor");
    c.gps_cov_scale_fixed = d("gps_cov_scale_fixed");
    c.gps_cov_scale_float = d("gps_cov_scale_float");
    c.float_meas_sigma = d("float_meas_sigma");
    c.float_bias_prior_sigma = d("float_bias_prior_sigma");
    c.float_bias_rw_sigma = d("float_bias_rw_sigma");
    c.float_bias_session_gap = d("float_bias_session_gap");
    c.nonrtk_sigma = d("nonrtk_sigma");
    c.gps_z_sigma_scale = d("gps_z_sigma_scale");
    c.gps_robust = b("gps_robust");
    c.gps_time_tol = d("gps_time_tol");
    c.gps_max_interp_gap = d("gps_max_interp_gap");
    c.gps_assoc_timeout = d("gps_assoc_timeout");
    c.align_min_travel = d("align_min_travel");
    c.align_min_travel_float = d("align_min_travel_float");
    c.align_min_pairs = i("align_min_pairs");
    c.align_max_rms = d("align_max_rms");
    c.align_max_rms_float = d("align_max_rms_float");
    c.enable_loop_closure = b("enable_loop_closure");
    c.sc.num_ring = i("sc_num_ring");
    c.sc.num_sector = i("sc_num_sector");
    c.sc.max_radius = d("sc_max_radius");
    c.sc.min_range = d("cloud_min_range");
    c.sc.lidar_height = d("sc_lidar_height");
    c.sc.dist_threshold = d("sc_dist_threshold");
    c.sc.num_candidates = i("sc_num_candidates");
    c.sc.exclude_recent = i("sc_exclude_recent");
    c.loop_min_query_gap = i("loop_min_query_gap");
    c.icp_method = get_parameter("icp_method").as_string();
    c.icp_voxel = d("icp_voxel");
    c.submap_size = i("submap_size");
    c.icp_max_corr = d("icp_max_corr");
    c.icp_min_fitness = d("icp_min_fitness");
    c.icp_max_rmse = d("icp_max_rmse");
    c.loop_sigma_rot = d("loop_sigma_rot");
    c.loop_sigma_trans = d("loop_sigma_trans");
    c.loop_robust_k = d("loop_robust_k");
    c.map_voxel = d("map_voxel");
    const auto lever = get_parameter("antenna_lever").as_double_array();
    if (lever.size() == 3)
      c.antenna_lever = Eigen::Vector3d(lever[0], lever[1], lever[2]);
    return c;
  }

  // Resolve static extrinsics from TF once the frame names are known.
  // body = odometry child frame (livox_frame for FAST-LIO2).
  // Retries for a while: on bag replay the static TF may arrive after the
  // first odometry message. Falls back to parameters after max_attempts.
  void resolveExtrinsics() {
    constexpr int kMaxAttempts = 200;  // ~ first 200 odom messages
    if ((antenna_resolved_ && base_resolved_) ||
        extrinsic_attempts_ > kMaxAttempts)
      return;
    ++extrinsic_attempts_;
    const std::string gps_frame = get_parameter("gps_frame").as_string();
    const std::string base_frame = get_parameter("base_frame").as_string();
    if (!antenna_resolved_) {
      try {
        const auto tf = tf_buffer_->lookupTransform(body_frame_, gps_frame,
                                                    tf2::TimePointZero);
        const Eigen::Vector3d l(tf.transform.translation.x,
                                tf.transform.translation.y,
                                tf.transform.translation.z);
        {
          std::lock_guard<std::mutex> lk(mtx_);
          backend_->setAntennaLever(l);
        }
        antenna_resolved_ = true;
        RCLCPP_INFO(get_logger(),
                    "antenna lever from TF %s->%s: [%.3f %.3f %.3f]",
                    body_frame_.c_str(), gps_frame.c_str(), l.x(), l.y(),
                    l.z());
      } catch (const tf2::TransformException&) {
        if (extrinsic_attempts_ == kMaxAttempts)
          RCLCPP_WARN(get_logger(),
                      "TF %s->%s unavailable, using antenna_lever param",
                      body_frame_.c_str(), gps_frame.c_str());
      }
    }
    if (!base_resolved_) {
      try {
        const auto tf = tf_buffer_->lookupTransform(body_frame_, base_frame,
                                                    tf2::TimePointZero);
        t_body_base_ = gtsam::Pose3(
            gtsam::Rot3::Quaternion(
                tf.transform.rotation.w, tf.transform.rotation.x,
                tf.transform.rotation.y, tf.transform.rotation.z),
            gtsam::Point3(tf.transform.translation.x,
                          tf.transform.translation.y,
                          tf.transform.translation.z));
        base_resolved_ = true;
        RCLCPP_INFO(get_logger(), "body->base from TF %s->%s",
                    body_frame_.c_str(), base_frame.c_str());
      } catch (const tf2::TransformException&) {
        if (extrinsic_attempts_ == kMaxAttempts) {
          const auto t = get_parameter("t_body_base").as_double_array();
          const auto q = get_parameter("q_body_base").as_double_array();
          if (t.size() == 3 && q.size() == 4) {
            t_body_base_ = gtsam::Pose3(
                gtsam::Rot3::Quaternion(q[0], q[1], q[2], q[3]),
                gtsam::Point3(t[0], t[1], t[2]));
          }
          RCLCPP_WARN(get_logger(),
                      "TF %s->%s unavailable, using t_body_base param",
                      body_frame_.c_str(), base_frame.c_str());
        }
      }
    }
  }

  // ------------------------------------------------------------------ inputs
  void cbSynced(const nav_msgs::msg::Odometry::ConstSharedPtr& odom,
                const sensor_msgs::msg::PointCloud2::ConstSharedPtr& cloud) {
    const double t = toSec(odom->header.stamp);
    if (odom_frame_.empty()) {
      odom_frame_ = odom->header.frame_id.empty() ? "odom"
                                                  : odom->header.frame_id;
      body_frame_ = odom->child_frame_id.empty() ? "livox_frame"
                                                 : odom->child_frame_id;
      RCLCPP_INFO(get_logger(), "frames: %s -> %s -> %s",
                  get_parameter("map_frame").as_string().c_str(),
                  odom_frame_.c_str(), body_frame_.c_str());
    }
    resolveExtrinsics();
    const gtsam::Pose3 t_ob = fromMsg(odom->pose.pose);

    gtsam::Pose3 t_map_odom, t_enu_map;
    bool need_kf;
    {
      std::lock_guard<std::mutex> lk(mtx_);
      last_odom_t_ = t;
      t_map_odom = backend_->mapToOdom();
      t_enu_map = enuMapTransform();
      need_kf = backend_->needsKeyframe(t_ob);
    }

    publishCorrected(t_enu_map, t_map_odom, t_ob, odom);

    if (!need_kf) return;
    auto pcl_cloud = std::make_shared<CloudT>();
    pcl::fromROSMsg(*cloud, *pcl_cloud);
    if (get_parameter("cloud_frame_mode").as_string() == "odom") {
      // /cloud_registered is in the odom frame: bring it back to the body
      CloudT tmp;
      pcl::transformPointCloud(*pcl_cloud, tmp,
                               t_ob.inverse().matrix().cast<float>());
      *pcl_cloud = tmp;
    }
    gtsam::Pose3 opt_pose, t_enu_map_kf;
    {
      std::lock_guard<std::mutex> lk(mtx_);
      const int idx = backend_->addKeyframe(t, t_ob, pcl_cloud, last_odom_t_);
      opt_pose = backend_->optimizedPoses()[idx];
      t_enu_map_kf = enuMapTransform();
    }
    publishKeyframe(t_enu_map_kf, opt_pose, odom->header.stamp);
    publishPath();
  }

  void cbNavPvt(const ublox_msgs::msg::NavPVT::ConstSharedPtr& msg) {
    // carrier solution: flags bits 6-7 (0x40 float, 0x80 fixed)
    if (msg->fix_type < 3 || !(msg->flags & 0x01)) return;  // need 3D+gnssOK
    GpsQuality q = GpsQuality::kOther;
    const uint8_t carr = msg->flags & 0xC0;
    if (carr == 0x80) q = GpsQuality::kFixed;
    else if (carr == 0x40) q = GpsQuality::kFloat;
    const double h_acc = msg->h_acc * 1e-3;  // mm -> m
    const double v_acc = msg->v_acc * 1e-3;
    // NavPVT has no header: stamp with node time (== bag time under sim time)
    const double t = now().seconds();
    std::lock_guard<std::mutex> lk(mtx_);
    const double now_s = last_odom_t_ > 0.0 ? last_odom_t_ : t;
    backend_->addGpsFix(t, msg->lat * 1e-7, msg->lon * 1e-7,
                        msg->height * 1e-3,
                        Eigen::Vector3d(h_acc, h_acc, v_acc), q, now_s);
  }

  // 'fused' mode: NavPVT carries only the RTK carrier-solution flag, which we
  // cache and pair with the next NavSatFix (both are published per GNSS epoch).
  void cbNavPvtFlag(const ublox_msgs::msg::NavPVT::ConstSharedPtr& msg) {
    GpsQuality q = GpsQuality::kOther;
    if (msg->fix_type >= 3 && (msg->flags & 0x01)) {  // 3D + gnssFixOK
      const uint8_t carr = msg->flags & 0xC0;
      if (carr == 0x80) q = GpsQuality::kFixed;
      else if (carr == 0x40) q = GpsQuality::kFloat;
    }
    std::lock_guard<std::mutex> lk(mtx_);
    last_navpvt_q_ = q;
    last_navpvt_t_ = now().seconds();
  }

  void cbNavSatFix(const sensor_msgs::msg::NavSatFix::ConstSharedPtr& msg) {
    if (msg->status.status < sensor_msgs::msg::NavSatStatus::STATUS_FIX)
      return;
    const double sx = std::sqrt(std::max(msg->position_covariance[0], 1e-12));
    const double sy = std::sqrt(std::max(msg->position_covariance[4], 1e-12));
    const double sz = std::sqrt(std::max(msg->position_covariance[8], 1e-12));
    const double t = now().seconds();  // node time == bag time under sim time
    std::lock_guard<std::mutex> lk(mtx_);
    GpsQuality q;
    if (gps_input_mode_ == "fused") {
      // Quality from the paired NavPVT flag (fresh within ~1 epoch). If no
      // recent flag, treat as non-RTK.
      q = (t - last_navpvt_t_ < 0.5) ? last_navpvt_q_ : GpsQuality::kOther;
    } else {  // navsatfix: classify by covariance thresholds
      const double h = std::max(sx, sy);
      q = GpsQuality::kOther;
      if (h <= get_parameter("fixed_max_std").as_double()) q = GpsQuality::kFixed;
      else if (h <= get_parameter("float_max_std").as_double())
        q = GpsQuality::kFloat;
    }
    const double now_s = last_odom_t_ > 0.0 ? last_odom_t_ : t;
    backend_->addGpsFix(t, msg->latitude, msg->longitude, msg->altitude,
                        Eigen::Vector3d(sx, sy, sz), q, now_s);
  }

  // ------------------------------------------------------------ loop closure
  void loopWorker() {
    const double period = get_parameter("loop_period").as_double();
    const int max_per_tick =
        static_cast<int>(get_parameter("loop_max_per_tick").as_int());
    while (running_) {
      std::this_thread::sleep_for(std::chrono::duration<double>(period));
      for (int k = 0; k < max_per_tick && running_; ++k) {
        std::optional<LoopTask> task;
        {
          std::lock_guard<std::mutex> lk(mtx_);
          task = backend_->nextLoopTask();
        }
        if (!task) break;
        auto result = backend_->verifyLoop(*task);  // slow ICP, lock-free
        if (!result) continue;
        {
          std::lock_guard<std::mutex> lk(mtx_);
          backend_->commitLoop(*result);
        }
        publishPath();
        publishLoopMarkers();
      }
    }
  }

  // ---------------------------------------------------------------- outputs
  // T_enu_map: maps a pose from the internal PGO map frame (FAST-LIO start pose)
  // into the ENU/datum frame using the estimated yaw+translation alignment, so
  // the published `map_frame` is world-aligned like the EKF's global output.
  // Identity until alignment converges (or if publish_enu is false), in which
  // case output stays in the LIO-start frame. Call with mtx_ held.
  gtsam::Pose3 enuMapTransform() {
    if (!get_parameter("publish_enu").as_bool()) return gtsam::Pose3();
    const auto& a = backend_->alignment();
    if (!a) return gtsam::Pose3();
    const gtsam::Pose3 t_map_enu(gtsam::Rot3::Rz(a->yaw),
                                 gtsam::Point3(a->t.x(), a->t.y(), a->z_offset));
    return t_map_enu.inverse();
  }

  void publishCorrected(const gtsam::Pose3& t_enu_map,
                        const gtsam::Pose3& t_map_odom,
                        const gtsam::Pose3& t_ob,
                        const nav_msgs::msg::Odometry::ConstSharedPtr& in) {
    const gtsam::Pose3 t_out_base = t_enu_map * t_map_odom * t_ob * t_body_base_;
    nav_msgs::msg::Odometry out;
    out.header.stamp = in->header.stamp;
    out.header.frame_id = get_parameter("map_frame").as_string();
    out.child_frame_id = get_parameter("base_frame").as_string();
    toMsg(t_out_base, out.pose.pose);
    out.twist = in->twist;
    pub_odom_->publish(out);

    if (get_parameter("publish_tf").as_bool()) {
      const gtsam::Pose3 t_out_odom = t_enu_map * t_map_odom;
      geometry_msgs::msg::TransformStamped tf;
      tf.header.stamp = in->header.stamp;
      tf.header.frame_id = out.header.frame_id;
      tf.child_frame_id = odom_frame_;
      const auto t = t_out_odom.translation();
      const auto q = t_out_odom.rotation().toQuaternion();
      tf.transform.translation.x = t.x();
      tf.transform.translation.y = t.y();
      tf.transform.translation.z = t.z();
      tf.transform.rotation.w = q.w();
      tf.transform.rotation.x = q.x();
      tf.transform.rotation.y = q.y();
      tf.transform.rotation.z = q.z();
      tf_broadcaster_->sendTransform(tf);
    }
  }

  void publishKeyframe(const gtsam::Pose3& t_enu_map, const gtsam::Pose3& pose,
                       const builtin_interfaces::msg::Time& stamp) {
    nav_msgs::msg::Odometry out;
    out.header.stamp = stamp;
    out.header.frame_id = get_parameter("map_frame").as_string();
    out.child_frame_id = body_frame_;
    toMsg(t_enu_map * pose, out.pose.pose);
    pub_pose_->publish(out);
  }

  void publishPath() {
    nav_msgs::msg::Path path;
    path.header.frame_id = get_parameter("map_frame").as_string();
    path.header.stamp = now();
    {
      std::lock_guard<std::mutex> lk(mtx_);
      const gtsam::Pose3 t_enu_map = enuMapTransform();
      const auto& kfs = backend_->keyframes();
      const auto& poses = backend_->optimizedPoses();
      path.poses.reserve(kfs.size());
      for (size_t i = 0; i < kfs.size(); ++i) {
        geometry_msgs::msg::PoseStamped ps;
        ps.header.frame_id = path.header.frame_id;
        ps.header.stamp.sec = static_cast<int32_t>(kfs[i].t);
        ps.header.stamp.nanosec = static_cast<uint32_t>(
            (kfs[i].t - static_cast<int64_t>(kfs[i].t)) * 1e9);
        toMsg(t_enu_map * poses[i], ps.pose);
        path.poses.push_back(std::move(ps));
      }
    }
    pub_path_->publish(path);
  }

  void publishLoopMarkers() {
    visualization_msgs::msg::Marker m;
    m.header.frame_id = get_parameter("map_frame").as_string();
    m.header.stamp = now();
    m.ns = "loops";
    m.id = 0;
    m.type = visualization_msgs::msg::Marker::LINE_LIST;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.scale.x = 0.15;
    m.color.r = 0.1;
    m.color.g = 0.9;
    m.color.b = 0.2;
    m.color.a = 1.0;
    m.pose.orientation.w = 1.0;
    {
      std::lock_guard<std::mutex> lk(mtx_);
      for (const auto& [c, q] : backend_->loops()) {
        for (int idx : {c, q}) {
          const auto t = backend_->optimizedPoses()[idx].translation();
          geometry_msgs::msg::Point p;
          p.x = t.x();
          p.y = t.y();
          p.z = t.z();
          m.points.push_back(p);
        }
      }
    }
    pub_markers_->publish(m);
  }

  void publishMap() {
    CloudT::Ptr map;
    {
      std::lock_guard<std::mutex> lk(mtx_);
      map = backend_->buildMap();
    }
    if (map->empty()) return;
    sensor_msgs::msg::PointCloud2 msg;
    pcl::toROSMsg(*map, msg);
    msg.header.frame_id = get_parameter("map_frame").as_string();
    msg.header.stamp = now();
    pub_map_->publish(msg);
  }

  void cbSave(const std_srvs::srv::Trigger::Request::SharedPtr,
              std_srvs::srv::Trigger::Response::SharedPtr res) {
    std::string dir = get_parameter("save_directory").as_string();
    if (!dir.empty() && dir[0] == '~') {
      const char* home = std::getenv("HOME");
      if (home) dir = std::string(home) + dir.substr(1);
    }
    try {
      std::vector<std::string> files;
      {
        std::lock_guard<std::mutex> lk(mtx_);
        files = backend_->save(dir);
      }
      publishMap();
      res->success = true;
      res->message = "Saved:";
      for (const auto& f : files) res->message += " " + f;
    } catch (const std::exception& e) {
      res->success = false;
      res->message = std::string("Save failed: ") + e.what();
    }
  }

  // ------------------------------------------------------------------ state
  std::unique_ptr<PGOBackend> backend_;
  std::mutex mtx_;
  std::string odom_frame_, body_frame_, gps_input_mode_;
  gtsam::Pose3 t_body_base_;
  double last_odom_t_ = -1.0;
  // fused-mode: latest NavPVT carrier-solution quality, paired with NavSatFix
  GpsQuality last_navpvt_q_ = GpsQuality::kOther;
  double last_navpvt_t_ = -1.0;
  bool antenna_resolved_ = false;
  bool base_resolved_ = false;
  int extrinsic_attempts_ = 0;
  std::atomic<bool> running_{true};
  std::thread loop_thread_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  message_filters::Subscriber<nav_msgs::msg::Odometry> sub_odom_;
  message_filters::Subscriber<sensor_msgs::msg::PointCloud2> sub_cloud_;
  std::unique_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;
  rclcpp::Subscription<ublox_msgs::msg::NavPVT>::SharedPtr sub_navpvt_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr sub_fix_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_, pub_pose_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_map_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_markers_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_save_;
  rclcpp::TimerBase::SharedPtr map_timer_;
};

}  // namespace lidar_gps_pgo

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<lidar_gps_pgo::PGONode>();
  rclcpp::executors::MultiThreadedExecutor exec(rclcpp::ExecutorOptions(), 2);
  exec.add_node(node);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}
