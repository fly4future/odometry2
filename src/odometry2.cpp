/* includes//{*/

#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/convert.h>
#include <tf2/exceptions.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include <fog_msgs/srv/get_px4_param_int.hpp>
#include <fog_msgs/srv/set_px4_param_int.hpp>
#include <fog_msgs/srv/get_px4_param_float.hpp>
#include <fog_msgs/srv/set_px4_param_float.hpp>
#include <fog_msgs/srv/change_odometry.hpp>

#include <fog_msgs/msg/control_interface_diagnostics.hpp>

#include <fog_lib/scope_timer.h>

#include <px4_msgs/msg/vehicle_odometry.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>  // This has to be here otherwise you will get cryptic linker error about missing function 'getTimestamp'

#include <nav_msgs/msg/odometry.hpp>

#include <std_msgs/msg/string.hpp>

#include "odometry/enums.h"
#include "types.h"
#include "odometry_utils.h"
#include "lateral_estimator.h"

/*//}*/

typedef std::tuple<std::string, int>   px4_int;
typedef std::tuple<std::string, float> px4_float;


using namespace std::placeholders;
using namespace fog_lib;

namespace odometry2
{

/* class Odometry2 //{ */
class Odometry2 : public rclcpp::Node {
public:
  Odometry2(rclcpp::NodeOptions options);

private:
  std::atomic_bool is_initialized_ = false;

  // | ----------------- Scope timer parameters ----------------- |
  bool             scope_timer_enable_   = false;
  rclcpp::Duration scope_timer_throttle_ = rclcpp::Duration(std::chrono::seconds(1));
  rclcpp::Duration scope_timer_min_dur_  = rclcpp::Duration::from_seconds(0.020);

  // | ------------------------ TF frames ----------------------- |
  std::string uav_name_            = "";
  std::string utm_origin_frame_    = "";
  std::string local_origin_frame_  = "";
  std::string ned_origin_frame_    = "";
  std::string frd_fcu_frame_       = "";
  std::string fcu_frame_           = "";
  std::string hector_origin_frame_ = "";
  std::string hector_frame_        = "";

  // | ---------------------- TF variables ---------------------- |
  std::shared_ptr<tf2_ros::Buffer>                     tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener>          tf_listener_;
  std::shared_ptr<tf2_ros::TransformBroadcaster>       tf_broadcaster_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
  geometry_msgs::msg::TransformStamped                 tf_local_origin_to_ned_origin_frame_;
  std::atomic_bool                                     publishing_static_tf_ = false;

  // | ---------------------- PX parameters --------------------- |
  std::vector<px4_int>   px4_params_int_;
  std::vector<px4_float> px4_params_float_;
  std::atomic_bool       set_initial_px4_params_                = false;
  std::atomic_bool       getting_px4_utm_position_              = false;
  std::atomic_bool       getting_control_interface_diagnostics_ = false;
  int                    px4_param_set_timeout_                 = 0;

  float      px4_position_[3];
  float      px4_orientation_[4];
  std::mutex px4_pose_mutex_;

  // | -------------------- PX EKF parameters ------------------- |
  int ekf_default_hgt_mode_ = 0;
  int ekf_default_mask_     = 0;

  // | -------------------- PX home position -------------------- |
  double           px4_utm_position_[2];
  std::atomic_bool republish_static_tf_ = false;
  std::mutex       px4_utm_position_mutex_;

  // | --------------------- GPS parameters --------------------- |
  std::atomic_bool gps_active_ = false;
  std::mutex       gps_raw_mutex_;
  float            gps_eph_;
  /* float            pos_gps_offset_[2]; */

  int   c_gps_init_msgs_       = 0;
  float gps_eph_max_           = 0;
  int   gps_msg_good_          = 0;
  int   gps_msg_err_           = 0;
  int   c_gps_eph_err_         = 0;
  int   c_gps_eph_good_        = 0;
  int   gps_num_init_msgs_     = 0;
  float gps_msg_interval_max_  = 0.0;
  float gps_msg_interval_warn_ = 0.0;

  // | -------------------- Hector parameters ------------------- |
  std::atomic_bool getting_hector_ = false;
  std::atomic_bool hector_active_  = false;
  std::mutex       hector_raw_mutex_;

  int    c_hector_init_msgs_         = 0;
  int    hector_num_init_msgs_       = 0;
  int    hector_default_ekf_mask_    = 0;
  int    hector_default_hgt_mode_    = 0;
  float  hector_msg_interval_max_    = 0.0;
  float  hector_msg_interval_warn_   = 0.0;
  double hector_hdg_previous_        = 0.0;
  float  hector_reset_response_wait_ = 0.0;
  float  hector_reset_wait_          = 0.0;
  float  hector_fusion_wait_         = 0.0;
  float  hector_max_position_jump_   = 0.0;
  float  hector_max_velocity_        = 0.0;

  float            current_visual_odometry_[2];
  float            hector_position_[3];
  float            hector_position_raw_prev_[2];
  float            hector_position_raw_[2];
  float            hector_orientation_[4];
  float            pos_orig_hector_[3];
  float            ori_orig_hector_[4];
  std::atomic_bool hector_tf_setup_ = false;
  std::atomic_bool published_hector = false;

  std::chrono::time_point<std::chrono::high_resolution_clock> time_odometry_timer_prev_;
  std::atomic_bool                                            time_odometry_timer_set_  = false;
  std::chrono::time_point<std::chrono::system_clock>          hector_reset_called_time_ = std::chrono::system_clock::now();  // Last hector reset time

  // | -------------------- PX Vision Message ------------------- |
  unsigned long long                                          timestamp_;
  std::int64_t                                                timestamp_raw_;
  std::chrono::time_point<std::chrono::high_resolution_clock> time_sync_time_;
  std::mutex                                                  timestamp_mutex_;

  // | -------------------- Lateral estimator ------------------- |

  std::mutex hector_lat_estimator_mutex_, hector_lat_position_mutex_;

  std::shared_ptr<LateralEstimator> hector_lat_estimator_;
  std::vector<lat_R_t>              R_lat_vec_;

  // | --------------------- Callback groups -------------------- |
  // a shared pointer to each callback group has to be saved or the callbacks will never get called
  std::vector<rclcpp::CallbackGroup::SharedPtr> callback_groups_;
  // new callback groups have to be initialized using this function to be saved into callback_groups_
  rclcpp::CallbackGroup::SharedPtr new_cbk_grp();

  // | ----------------------- Publishers ----------------------- |
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr            local_odom_publisher_;
  rclcpp::Publisher<fog_msgs::msg::OdometryDiagnostics>::SharedPtr diagnostics_publisher_;

  // | ----------------------- Subscribers ---------------------- |
  rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr             pixhawk_odom_subscriber_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr            hector_pose_subscriber_;
  rclcpp::Subscription<px4_msgs::msg::Timesync>::SharedPtr                    timesync_subscriber_;
  rclcpp::Subscription<px4_msgs::msg::VehicleGpsPosition>::SharedPtr          gps_subscriber_;
  rclcpp::Subscription<fog_msgs::msg::ControlInterfaceDiagnostics>::SharedPtr control_interface_diagnostics_subscriber_;

  // | -------------------- Service providers ------------------- |
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr        reset_hector_service_;
  rclcpp::Service<fog_msgs::srv::ChangeOdometry>::SharedPtr change_odometry_source_;

  // | --------------------- Service clients -------------------- |
  rclcpp::Client<fog_msgs::srv::SetPx4ParamInt>::SharedPtr   set_px4_param_int_;
  rclcpp::Client<fog_msgs::srv::SetPx4ParamFloat>::SharedPtr set_px4_param_float_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr          reset_hector_client_;

  // | ------------------ Subscriber callbacks ------------------ |
  void pixhawkOdomCallback(const px4_msgs::msg::VehicleOdometry::UniquePtr msg);
  void hectorPoseCallback(const geometry_msgs::msg::PoseStamped::UniquePtr msg);
  void timesyncCallback(const px4_msgs::msg::Timesync::UniquePtr msg);
  void gpsCallback(const px4_msgs::msg::VehicleGpsPosition::UniquePtr msg);
  void ControlInterfaceDiagnosticsCallback(const fog_msgs::msg::ControlInterfaceDiagnostics::UniquePtr msg);

  // | -------------------------- Utils ------------------------- |
  bool setInitialPx4Params();

  // | ----------------------- Publishing ----------------------- |
  void publishStaticTF();
  void publishLocalOdomAndTF();

  // --------------------------------------------------------------
  // |                     Internal functions                     |
  // --------------------------------------------------------------
  bool checkHectorReliability();
  bool checkGpsReliability();
  bool isValidType(const fog_msgs::msg::OdometryType &type);

  // --------------------------------------------------------------
  // |                          Routines                          |
  // --------------------------------------------------------------

  // | ------------------ Odometry node routine ----------------- |
  std::recursive_mutex state_mutex_;
  odometry_state_t     odometry_state_      = odometry_state_t::not_connected;
  std::atomic_bool     odometry_update_ekf_ = false;

  rclcpp::TimerBase::SharedPtr odometry_timer_;

  double odometry_loop_rate_;
  void   update_odometry_state();
  void   odometryRoutine(void);

  void state_odometry_not_connected();
  void state_odometry_init();
  void state_odometry_gps();
  void state_odometry_hector();
  void state_odometry_missing_odometry();

  void publishOdometryDiagnostics();

  // | ------------------ GPS estimator routine ----------------- |
  std::recursive_mutex gps_mutex_;
  estimator_state_t    gps_state_ = estimator_state_t_::init;

  rclcpp::TimerBase::SharedPtr gps_timer_;

  double gps_loop_rate_;
  void   update_gps_state();
  void   gpsRoutine(void);

  void state_gps_init();
  void state_gps_reliable();
  void state_gps_not_reliable();
  void state_gps_inactive();

  void publishGpsDiagnostics();

  // | ------------------ Hector estimator routine ----------------- |
  std::recursive_mutex hector_mutex_;
  estimator_state_t    hector_state_ = estimator_state_t_::init;

  rclcpp::TimerBase::SharedPtr hector_timer_;

  double hector_loop_rate_;
  void   update_hector_state();
  void   hectorRoutine(void);

  void state_hector_init();
  void state_hector_reliable();
  void state_hector_not_reliable();
  void state_hector_restart();

  void publishHectorDiagnostics();

  // | -------------------- Routine handling -------------------- |
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  /*//}*/

  // | --------------------- Utils function --------------------- |
  template <typename T, typename U, typename V>
  bool uploadPx4Parameters(const T &service_name, const U &param_array, const V &service_client);

  template <typename T>
  bool checkPx4ParamSetOutput(const std::shared_future<T> &f);
};
//}

/* constructor //{ */
Odometry2::Odometry2(rclcpp::NodeOptions options) : Node("odometry2", options) {

  RCLCPP_INFO(get_logger(), "[%s]: Initializing...", get_name());

  try {
    uav_name_ = std::string(std::getenv("DRONE_DEVICE_ID"));
  }
  catch (...) {
    RCLCPP_WARN(get_logger(), "[%s]: Environment variable DRONE_DEVICE_ID was not defined!", get_name());
  }
  RCLCPP_INFO(get_logger(), "[%s]: UAV name is: '%s'", get_name(), uav_name_.c_str());

  /* parse general params from config file //{ */
  RCLCPP_INFO(get_logger(), "-------------- Loading parameters --------------");
  bool loaded_successfully = true;

  loaded_successfully &= parse_param("odometry_loop_rate", odometry_loop_rate_, *this);

  loaded_successfully &= parse_param("gps.active", gps_active_, *this);
  loaded_successfully &= parse_param("gps.eph_max", gps_eph_max_, *this);
  loaded_successfully &= parse_param("gps.msg_good", gps_msg_good_, *this);
  loaded_successfully &= parse_param("gps.msg_err", gps_msg_err_, *this);
  loaded_successfully &= parse_param("gps.num_init_msgs", gps_num_init_msgs_, *this);

  loaded_successfully &= parse_param("hector.active", hector_active_, *this);
  loaded_successfully &= parse_param("hector.reset_wait", hector_reset_wait_, *this);
  loaded_successfully &= parse_param("hector.reset_response_wait_", hector_reset_response_wait_, *this);
  loaded_successfully &= parse_param("hector.num_init_msgs", hector_num_init_msgs_, *this);
  loaded_successfully &= parse_param("hector.msg_interval_max", hector_msg_interval_max_, *this);
  loaded_successfully &= parse_param("hector.msg_interval_warn", hector_msg_interval_warn_, *this);
  loaded_successfully &= parse_param("hector.max_position_jump", hector_max_position_jump_, *this);
  loaded_successfully &= parse_param("hector.max_velocity", hector_max_velocity_, *this);

  loaded_successfully &= parse_param("px4.param_set_timeout", px4_param_set_timeout_, *this);

  int   param_int;
  float param_float;

  loaded_successfully &= parse_param("px4.EKF2_AID_MASK", ekf_default_mask_, *this);
  px4_params_int_.push_back(px4_int("EKF2_AID_MASK", ekf_default_mask_));
  loaded_successfully &= parse_param("px4.EKF2_EV_NOISE_MD", param_int, *this);
  px4_params_int.push_back(px4_int("EKF2_EV_NOISE_MD", param_int));
  loaded_successfully &= parse_param("px4.EKF2_RNG_AID", param_int, *this);
  px4_params_int_.push_back(px4_int("EKF2_RNG_AID", param_int));
  loaded_successfully &= parse_param("px4.EKF2_HGT_MODE", param_int, *this);
  px4_params_int_.push_back(px4_int("EKF2_HGT_MODE", param_int));
  loaded_successfully &= parse_param("px4.EKF2_RNG_A_HMAX", param_float, *this);
  px4_params_float_.push_back(px4_float("EKF2_RNG_A_HMAX", param_float));

  loaded_successfully &= parse_param("px4.EKF2_EV_DELAY", param_float, *this);
  px4_params_float.push_back(px4_float("EKF2_EV_DELAY", param_float));
  loaded_successfully &= parse_param("px4.EKF2_EVP_NOISE", param_float, *this);
  px4_params_float.push_back(px4_float("EKF2_EVP_NOISE", param_float));
  loaded_successfully &= parse_param("px4.EKF2_EVV_NOISE", param_float, *this);
  px4_params_float.push_back(px4_float("EKF2_EVV_NOISE", param_float));
  loaded_successfully &= parse_param("px4.EKF2_RNG_A_HMAX", param_float, *this);
  px4_params_float.push_back(px4_float("EKF2_RNG_A_HMAX", param_float));
  loaded_successfully &= parse_param("px4.MPC_XY_CRUISE", param_float, *this);
  px4_params_float.push_back(px4_float("MPC_XY_CRUISE", param_float));
  loaded_successfully &= parse_param("px4.MC_YAWRATE_MAX", param_float, *this);
  px4_params_float.push_back(px4_float("MC_YAWRATE_MAX", param_float));
  loaded_successfully &= parse_param("px4.MPC_ACC_HOR", param_float, *this);
  px4_params_float.push_back(px4_float("MPC_ACC_HOR", param_float));
  loaded_successfully &= parse_param("px4.MPC_ACC_HOR_MAX", param_float, *this);
  px4_params_float.push_back(px4_float("MPC_ACC_HOR_MAX", param_float));
  loaded_successfully &= parse_param("px4.MPC_JERK_AUTO", param_float, *this);
  px4_params_float.push_back(px4_float("MPC_JERK_AUTO", param_float));
  loaded_successfully &= parse_param("px4.MPC_JERK_MAX", param_float, *this);
  px4_params_float.push_back(px4_float("MPC_JERK_MAX", param_float));
  loaded_successfully &= parse_param("px4.MPC_ACC_DOWN_MAX", param_float, *this);
  px4_params_float.push_back(px4_float("MPC_ACC_DOWN_MAX", param_float));
  loaded_successfully &= parse_param("px4.MPC_ACC_UP_MAX", param_float, *this);
  px4_params_float.push_back(px4_float("MPC_ACC_UP_MAX", param_float));

  if (!loaded_successfully) {
    const std::string str = "Could not load all non-optional parameters. Shutting down.";
    RCLCPP_ERROR(get_logger(), "%s", str.c_str());
    rclcpp::shutdown();
    return;
  }
  //}

  // | -------------------- Frame definition -------------------- |
  utm_origin_frame_    = "utm_origin";                 // ENU frame (East-North-Up)
  local_origin_frame_  = uav_name_ + "/local_origin";  // ENU frame (East-North-Up)
  fcu_frame_           = uav_name_ + "/fcu";           // FLU frame (Front-Left-Up) match also ENU frame (East-North-Up)
  frd_fcu_frame_       = uav_name_ + "/frd_fcu";       // FRD frame (Front-Right-Down)
  ned_origin_frame_    = uav_name_ + "/ned_origin";    // NED frame (North-East-Down)
  hector_origin_frame_ = uav_name_ + "/hector_origin";
  hector_frame_        = uav_name_ + "/hector_fcu";

  // | -------------------- Lateral estimator ------------------- |

  /* lateral process covariance (Q matrix) //{ */

  lat_Q_t Q_lat = lat_Q_t::Identity();
  double  lat_pos, lat_vel, lat_acc;
  parse_param("lateral.process_covariance.pos", lat_pos, *this);
  Q_lat(LAT_POS_X, LAT_POS_X) *= lat_pos;
  Q_lat(LAT_POS_Y, LAT_POS_Y) *= lat_pos;
  parse_param("lateral.process_covariance.vel", lat_vel, *this);
  Q_lat(LAT_VEL_X, LAT_VEL_X) *= lat_vel;
  Q_lat(LAT_VEL_Y, LAT_VEL_Y) *= lat_vel;
  parse_param("lateral.process_covariance.acc", lat_acc, *this);
  Q_lat(LAT_ACC_X, LAT_ACC_X) *= lat_acc;
  Q_lat(LAT_ACC_Y, LAT_ACC_Y) *= lat_acc;

  //}

  hector_lat_estimator_ = std::make_shared<LateralEstimator>("hector", Q_lat, R_lat_vec_);

  // | ----------------------- Publishers ----------------------- |
  rclcpp::QoS qos(rclcpp::KeepLast(3));
  local_odom_publisher_  = this->create_publisher<nav_msgs::msg::Odometry>("~/local_odom_out", qos);
  diagnostics_publisher_ = this->create_publisher<fog_msgs::msg::OdometryDiagnostics>("~/diagnostics_out", qos);

  // | ----------------------- Subscribers ---------------------- |
  rclcpp::SubscriptionOptions subopts;

  subopts.callback_group                    = new_cbk_grp();
  pixhawk_odom_subscriber_                  = this->create_subscription<px4_msgs::msg::VehicleOdometry>("~/pixhawk_odom_in", rclcpp::SystemDefaultsQoS(),
                                                                                       std::bind(&Odometry2::pixhawkOdomCallback, this, _1), subopts);
  subopts.callback_group                    = new_cbk_grp();
  timesync_subscriber_                      = this->create_subscription<px4_msgs::msg::Timesync>("~/timesync_in", rclcpp::SystemDefaultsQoS(),
                                                                            std::bind(&Odometry2::timesyncCallback, this, _1), subopts);
  subopts.callback_group                    = new_cbk_grp();
  gps_subscriber_                           = this->create_subscription<px4_msgs::msg::VehicleGpsPosition>("~/gps_in", rclcpp::SystemDefaultsQoS(),
                                                                                 std::bind(&Odometry2::gpsCallback, this, _1), subopts);
  subopts.callback_group                    = new_cbk_grp();
  control_interface_diagnostics_subscriber_ = this->create_subscription<fog_msgs::msg::ControlInterfaceDiagnostics>(
      "~/control_interface_diagnostics_in", rclcpp::SystemDefaultsQoS(), std::bind(&Odometry2::ControlInterfaceDiagnosticsCallback, this, _1), subopts);
  subopts.callback_group  = new_cbk_grp();
  hector_pose_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("~/hector_pose_in", rclcpp::SystemDefaultsQoS(),
                                                                                       std::bind(&Odometry2::hectorPoseCallback, this, _1), subopts);

  // | -------------------- Service clients  -------------------- |
  set_px4_param_int_   = this->create_client<fog_msgs::srv::SetPx4ParamInt>("~/set_px4_param_int");
  set_px4_param_float_ = this->create_client<fog_msgs::srv::SetPx4ParamFloat>("~/set_px4_param_float");
  reset_hector_client_ = this->create_client<std_srvs::srv::Trigger>("~/reset_hector_service_local_out");

  // | -------------------- Service handlers -------------------- |
  const auto qos_profile  = qos.get_rmw_qos_profile();
  const auto srv_grp_ptr  = new_cbk_grp();
  reset_hector_service_   = this->create_service<std_srvs::srv::Trigger>("~/reset_hector_service_local_in",
                                                                       std::bind(&Odometry2::resetHectorCallback, this, _1, _2), qos_profile, srv_grp_ptr);
  change_odometry_source_ = this->create_service<fog_msgs::srv::ChangeOdometry>(
      "~/change_odometry_source", std::bind(&Odometry2::changeOdometryCallback, this, _1, _2), qos_profile, srv_grp_ptr);

  callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  odometry_timer_ =
      this->create_wall_timer(std::chrono::duration<double>(1.0 / odometry_loop_rate_), std::bind(&Odometry2::odometryRoutine, this), callback_group_);

  tf_broadcaster_        = nullptr;
  static_tf_broadcaster_ = nullptr;

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_buffer_->setUsingDedicatedThread(true);
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this, false);

  is_initialized_ = true;
  RCLCPP_INFO(get_logger(), "[%s]: Initialized", get_name());
}
//}

// --------------------------------------------------------------
// |                    Subscriber Callbacks                    |
// --------------------------------------------------------------

/* timesyncCallback //{ */
void Odometry2::timesyncCallback(const px4_msgs::msg::Timesync::UniquePtr msg) {
  if (!is_initialized_) {
    return;
  }
  if (msg->sys_id == 1) {  // TODO: System id might not be the same all the time. Maybe define with param?
    std::scoped_lock lock(timestamp_mutex_);
    timestamp_.store(msg->ts1);
    timestamp_raw_.store(msg->tc1);
    time_sync_time_ = std::chrono::high_resolution_clock::now();
  }
}
//}

/* hectorPoseCallback //{ */
void Odometry2::hectorPoseCallback(const geometry_msgs::msg::PoseStamped::UniquePtr msg) {

  // If not initialized
  if (!is_initialized_) {
    return;
  }

  RCLCPP_INFO_ONCE(get_logger(), "[%s]: Getting hector poses!", this->get_name());

  std::scoped_lock lock(hector_raw_mutex_);

  time_hector_last_msg_ = std::chrono::system_clock::now();

  /* Infinite value check//{*/
  if (!std::isfinite(msg->pose.position.x)) {
    RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "[%s]: not finite value detected in variable \"pose.position.x\" (hectorcallback) !!!",
                          this->get_name());
    return;
  }

  if (!std::isfinite(msg->pose.position.y)) {
    RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "[%s]: not finite value detected in variable \"pose.position.y\" (hectorCallback) !!!",
                          this->get_name());
    return;
  } /*//}*/

  /*Hector convergence//{*/
  // Wait for hector to converge
  if (c_hector_init_msgs_++ < hector_num_init_msgs_) {
    RCLCPP_INFO(this->get_logger(), "[%s]: Hector pose #%d - x: %f y: %f", this->get_name(), c_hector_init_msgs_, msg->pose.position.x, msg->pose.position.y);
    hector_position_raw_[0] = msg->pose.position.x;
    hector_position_raw_[1] = msg->pose.position.y;
    return;
  } /*//}*/

  /* RCLCPP_INFO(this->get_logger(), "[%s]: Hector pose - x: %f y: %f", this->get_name(), msg->pose.position.x, msg->pose.position.y); */

  // Update previous hector position for the reliability check
  hector_position_raw_prev_[0] = hector_position_raw_[0];
  hector_position_raw_prev_[1] = hector_position_raw_[1];

  // Save current hector position
  hector_position_raw_[0] = msg->pose.position.x;
  hector_position_raw_[1] = msg->pose.position.y;

  getting_hector_ = true;
}
//}

/* pixhawkOdomCallback //{ */
void Odometry2::pixhawkOdomCallback(const px4_msgs::msg::VehicleOdometry::UniquePtr msg) {

  std::scoped_lock lock(px4_pose_mutex_);

  if (!is_initialized_) {
    return;
  }

  px4_position_[0]    = msg->x;
  px4_position_[1]    = msg->y;
  px4_position_[2]    = msg->z;
  px4_orientation_[0] = msg->q[0];  // q.w
  px4_orientation_[1] = msg->q[1];  // q.x
  px4_orientation_[2] = msg->q[2];  // q.y
  px4_orientation_[3] = msg->q[3];  // q.z

  getting_pixhawk_odom_ = true;
  RCLCPP_INFO_ONCE(get_logger(), "[%s]: Getting pixhawk odometry!", get_name());
}
//}

/* gpsCallback //{ */
void Odometry2::gpsCallback(const px4_msgs::msg::VehicleGpsPosition::UniquePtr msg) {

  if (!is_initialized_) {
    return;
  }

  RCLCPP_INFO_ONCE(this->get_logger(), "[%s] Getting Vehicle GPS!", this->get_name());

  std::scoped_lock lock(gps_raw_mutex_);

  time_gps_last_msg_ = std::chrono::system_clock::now();

  // Waiting for gps convergence after bootup
  if (c_gps_init_msgs_++ < gps_num_init_msgs_) {
    RCLCPP_INFO(this->get_logger(), "[%s]: GPS pose #%d - lat: %f lon: %f", this->get_name(), c_gps_init_msgs_, msg->lat, msg->lon);
    c_gps_init_msgs_++;
    return;
  }

  // Save the EPH value to evaluate the GPS quality
  // TODO: Check the https://docs.px4.io/master/en/advanced_config/parameter_reference.html website, the EPH value might be set by default to trigger failsafe,
  // or GPS lost
  gps_eph_ = msg->eph;

  getting_gps_ = true;
}
//}

/* ControlInterfaceDiagnosticsCallback //{ */
void Odometry2::ControlInterfaceDiagnosticsCallback([[maybe_unused]] const fog_msgs::msg::ControlInterfaceDiagnostics::UniquePtr msg) {

  // Check if the Control Interface is in the valid state to cooperate with
  if (msg->vehicle_state.state == fog_msgs::msg::ControlInterfaceVehicleState::NOT_CONNECTED ||
      msg->vehicle_state.state == fog_msgs::msg::ControlInterfaceVehicleState::INVALID) {
    return;
  }

  getting_control_interface_diagnostics_ = true;
  RCLCPP_INFO_ONCE(get_logger(), "[%s]: Getting control diagnostics!", get_name());
}
//}

// --------------------------------------------------------------
// |                      Service Callbacks                     |
// --------------------------------------------------------------

/* changeOdometryCallback //{ */
bool Odometry2::changeOdometryCallback(const std::shared_ptr<fog_msgs::srv::ChangeOdometry::Request>  request,
                                       const std::shared_ptr<fog_msgs::srv::ChangeOdometry::Response> response) {

  if (!is_initialized_) {
    return false;
  }

  // Check whether a valid type was requested
  if (!isValidType(request->odometry_type)) {
    RCLCPP_ERROR(this->get_logger(), "[%s]: %d is not a valid odometry type", this->get_name(), request->odometry_type.type);
    response->success = false;
    response->message = ("Not a valid odometry type");
    return false;
  }

  std::scoped_lock lock(gps_state_, hector_state_);

  // Change the estimator
  if (request->odometry_type.type == 0) {
    gps_state_    = estimator_state_t::not_reliable;
    hector_state_ = estimator_state_t::not_reliable;
    RCLCPP_WARN(this->get_logger(), "[%s]: Changing odometry to automatic GPS or HECTOR", this->get_name());
  } else if (request->odometry_type.type == 1) {
    gps_state_    = estimator_state_t::not_reliable;
    hector_state_ = estimator_state_t::inactive;
    RCLCPP_WARN(this->get_logger(), "[%s]: Changing odometry to GPS only", this->get_name());
  } else if (request->odometry_type.type == 2) {
    gps_state_    = estimator_state_t::inactive;
    hector_state_ = estimator_state_t::not_reliable;
    RCLCPP_WARN(this->get_logger(), "[%s]: Changing odometry to HECTOR only", this->get_name());
  }

  response->success = true;
  response->message = "Done";

  return true;
}
//}

/* resetHectorCallback //{ */
bool Odometry2::resetHectorCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                    std::shared_ptr<std_srvs::srv::Trigger::Response>      response) {
  if (!is_initialized_) {
    response->message = "ROS not initialized";
    response->success = false;
    return true;
  }

  std::scoped_lock lock(hector_state_);

  if (hector_state_ == estimator_state_t::restart || hector_state_ == estimator_state_t::not_reliable) {
    response->message = "Hector in reset mode";
    response->success = false;
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500, "[Odometry2]: Hector is already in reset mode.");
    return true;
  }

  RCLCPP_INFO(this->get_logger(), "[Odometry2]: Hector set to restart state.");

  hector_state_ = estimator_state_t::not_reliable;

  response->message = "Hector set to restart state.";
  response->success = true;

  return true;
}
//}

// --------------------------------------------------------------
// |                   Odometry node routines                   |
// --------------------------------------------------------------

/* odometryRoutine//{ */
void Odometry2::odometryRoutine() {
  scope_timer      tim(scope_timer_enable_, "odometryRoutine", get_logger(), scope_timer_throttle_, scope_timer_min_dur_);
  std::scoped_lock lock(state_mutex_);

  const auto prev_state = odometry_state_;
  update_odometry_state();

  if (prev_state != odometry_state_)
    publishOdometryDiagnostics();

  //TODO: Udelat novej stav, tzv zmeny, ve kterem se provede prepnuti parametru pixhawku, tak, aby se to mohlo pokusit provest opakovane
  // Update EKF parameters for the new estimator
  if (odometry_update_ekf_) {
    if (updateEkfParameters()) {
      odometry_update_ekf_ = false;
    } else {
      RCLCPP_WARN(get_logger(), "Fail to set all PX4 parameters for new estimator. Repeat in next cycle.");
    }
  }
}
//}

/* odometryDiagnosticsRoutine(); //{ */
void Odometry2::odometryDiagnosticsRoutine() {
  scope_timer      tim(scope_timer_enable_, "odometryDiagnosticsRoutine", get_logger(), scope_timer_throttle_, scope_timer_min_dur_);
  std::scoped_lock lock(state_mutex_);

  // publish some diags
  publishOdometryDiagnostics();
}
//}

/* update_odometry_state//{ */
// the following mutexes have to be locked by the calling function:
// state_mutex_
void Odometry2::update_odometry_state() {

  // process the vehicle's state
  switch (odometry_state_) {
    case odometry_state_t::not_connected:
      state_odometry_not_connected();
      break;
    case odometry_state_t::init:
      state_odometry_init();
      break;
    case odometry_state_t::gps:
      state_odometry_gps();
      break;
    case odometry_state_t::hector:
      state_odometry_hector();
      break;
    case odometry_state_t::missing_odometry:
      state_odometry_missing_odometry();
      break;
    default:
      assert(false);
      RCLCPP_ERROR(get_logger(), "Invalid odometry state, this should never happen!");
      return;
  }
}
//}

/* state_odometry_not_connected//{ */
// the following mutexes have to be locked by the calling function:
// state_mutex_
void Odometry2::state_odometry_not_connected() {

  RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "Odometry state: Waiting to initialize Pixhawk and Control Interface.");

  if (getting_pixhawk_odom_ && getting_control_interface_diagnostics_) {
    RCLCPP_INFO(get_logger(), "[%s]: Pixhawk and Control Interface is ready", get_name());
    odometry_state_ = odometry_state_t::init;
  }
}
//}

/* state_odometry_init//{ */
// the following mutexes have to be locked by the calling function:
// state_mutex_
void Odometry2::state_odometry_init() {

  RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "Odometry state: Initializing odometry module.");

  // Set and handle initial PX4 parameters setting
  if (!set_initial_px4_params_) {
    RCLCPP_INFO(get_logger(), "Setting initial PX4 parameters.");
    if (setInitialPx4Params()) {
      set_initial_px4_params_ = true;
    } else {
      RCLCPP_WARN(get_logger(), "Fail to set all PX4 parameters. Will repeat in next cycle.");
      return;
    }
  }

  // Handle static TF initialization
  if (static_tf_broadcaster_ == nullptr) {
    static_tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this->shared_from_this());
    publishStaticTF();
  }

  // Wait until gps estimator is ready. GPS is required to initalize hector
  std::scoped_lock lock(gps_state_);

  if (gps_state_ == estimator_state_t::reliable) {
    odometry_state_ = odometry_state_t::gps;
    return;
  } else {
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "Odometry state: Waiting for GPS estimator.");
    return;
  }
}
//}

/* state_odometry_gps//{ */
// the following mutexes have to be locked by the calling function:
// state_mutex_
void Odometry2::state_odometry_gps() {

  RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "Odometry state: Using GPS odometry");

  std::scoped_lock lock(gps_state_);

  if (!checkEstimatorReliability()) {
    return;
  }

  publishGpsTF();
  publishGpsOdometry();
}
//}

/* state_odometry_hector//{ */
// the following mutexes have to be locked by the calling function:
// state_mutex_
void Odometry2::state_odometry_hector() {

  RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "Odometry state: Using hector odometry");

  std::scoped_lock lock(hector_state_);

  if (!checkEstimatorReliability()) {
    return;
  }

  publishHectorTF();
  publishHectorOdometry();


  // CALL LAND IF REQUIRED BY THE RULES OR FORCE HOVER UNTIL ODOMETRY RELIABLE ENOUGH
}
//}

/* state_odometry_missing_odometry//{ */
// the following mutexes have to be locked by the calling function:
// state_mutex_
void Odometry2::state_odometry_missing_odometry() {

  RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "Odometry state: Missing odometry");

  // CALL LAND IF REQUIRED BY THE RULES OR FORCE HOVER UNTIL ODOMETRY RELIABLE ENOUGH
}
//}

// --------------------------------------------------------------
// |                    GPS estimator routine                   |
// --------------------------------------------------------------

/* gpsRoutine//{ */
void Odometry2::gpsRoutine() {
  scope_timer      tim(scope_timer_enable_, "gpsRoutine", get_logger(), scope_timer_throttle_, scope_timer_min_dur_);
  std::scoped_lock lock(gps_mutex_);

  const auto prev_state = gps_state_;
  update_gps_state();
  if (prev_state != gps_state_)
    publishGpsDiagnostics();
}
//}

/* gpsDiagnosticsRoutine(); //{ */
void Odometry2::gpsDiagnosticsRoutine() {
  scope_timer      tim(scope_timer_enable_, "gpsDiagnosticsRoutine", get_logger(), scope_timer_throttle_, scope_timer_min_dur_);
  std::scoped_lock lock(gps_mutex_);

  // publish some diags
  publishGpsDiagnostics();
}
//}

/* update_gps_state//{ */
// the following mutexes have to be locked by the calling function:
// gps_mutex_
void Odometry2::update_gps_state() {

  // process the vehicle's state
  switch (gps_state_) {
    case estimator_state_t::init:
      state_gps_init();
      break;
    case estimator_state_t::reliable:
      state_gps_reliable();
      break;
    case estimator_state_t::not_reliable:
      state_gps_not_reliable();
      break;
    case estimator_state_t::inactive:
      state_gps_inactive();
      break;
    default:
      assert(false);
      RCLCPP_ERROR(get_logger(), "Invalid gps state, this should never happen!");
      return;
  }
}
//}

/* state_gps_init//{ */
// the following mutexes have to be locked by the calling function:
// gps_mutex_
void Odometry2::state_gps_init() {

  RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "GPS state: Initializing.");

  if (getting_pixhawk_odom_ && getting_gps_) {
    RCLCPP_INFO(get_logger(), "[%s]: GPS and pixhawk odometry received", get_name());
  } else {
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "Waiting to receive GPS and odometry from Pixhawk.");
  }

  // Check if the estimator should be active
  if (!gps_active_) {
    gps_state_ = estimator_state_t::inactive;
    return;
  }
  gps_state_ = estimator_state_t::reliable;
}
//}

/* state_gps_reliable//{ */
// the following mutexes have to be locked by the calling function:
// gps_mutex_
void Odometry2::state_gps_reliable() {

  RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "GPS state: Reliable");

  std::scoped_lock lock(gps_raw_mutex_);

  // Check gps reliability
  if (!checkGpsReliability()) {
    gps_state_ = estimator_state_t::not_reliable;
    return;
  }
}
//}

/* state_gps_not_reliable//{ */
// the following mutexes have to be locked by the calling function:
// gps_mutex_
void Odometry2::state_gps_not_reliable() {

  RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "GPS state: Not reliable.");

  // GPS quality is getting better
  if (msg->eph < gps_eph_max_) {
    if (c_gps_eph_good_++ >= gps_msg_good_) {

      RCLCPP_WARN(this->get_logger(), "[%s] GPS quality is good!", this->get_name());
      c_gps_eph_good_ = 0;
      gps_state_      = estimator_state_t::reliable;
      return;

    } else {
      RCLCPP_WARN(this->get_logger(), "[%s] GPS quality is improving! #%d EPH value: %f", this->get_name(), c_gps_eph_good_, msg->eph);
    }
  } else {
    c_gps_eph_good_ = 0;  // Reset the good message counter
  }
}
//}

/* state_gps_inactive//{ */
// the following mutexes have to be locked by the calling function:
// gps_mutex_
void Odometry2::state_gps_inactive() {

  RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "GPS state: Inactive.");
}
//}


// --------------------------------------------------------------
// |                  Hector estimator routine                  |
// --------------------------------------------------------------

/* hectorRoutine//{ */
void Odometry2::hectorRoutine() {
  scope_timer      tim(scope_timer_enable_, "hectorRoutine", get_logger(), scope_timer_throttle_, scope_timer_min_dur_);
  std::scoped_lock lock(hector_mutex_);

  const auto prev_state = hector_state_;
  update_hector_state();
  if (prev_state != hector_state_)
    publishHectorDiagnostics();
}
//}

/* hectorDiagnosticsRoutine(); //{ */
void Odometry2::hectorDiagnosticsRoutine() {
  scope_timer      tim(scope_timer_enable_, "hectorDiagnosticsRoutine", get_logger(), scope_timer_throttle_, scope_timer_min_dur_);
  std::scoped_lock lock(hector_mutex_);

  // publish some diags
  publishHectorDiagnostics();
}
//}

/* update_hector_state//{ */
// the following mutexes have to be locked by the calling function:
// hector_mutex_
void Odometry2::update_hector_state() {

  // process the vehicle's state
  switch (hector_state_) {
    case estimator_state_t::init:
      state_hector_init();
      break;
    case estimator_state_t::reliable:
      state_hector_reliable();
      break;
    case estimator_state_t::not_reliable:
      state_hector_not_reliable();
      break;
    case estimator_state_t::restart:
      state_hector_restart();
      break;
    default:
      assert(false);
      RCLCPP_ERROR(get_logger(), "Invalid hector state, this should never happen!");
      return;
  }
}
//}

/* state_hector_init//{ */
// the following mutexes have to be locked by the calling function:
// hector_mutex_
void Odometry2::state_hector_init() {

  if (!getting_hector_) {
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "Hector state: Waiting to receive hector data.");
    return;
  }

  // Wait to obtain pixhawk odom to attach hector tf accordingly
  if (!getting_pixhawk_odom_) {
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "[%s]: Waiting for pixhawk odom to initialize tf", this->get_name());
    return;
  }

  /* Setup hector_origin based on current gps_position */ /*//{*/
  if (publishing_static_tf_) {
    auto tf             = transformBetween(fcu_frame_, world_frame_);
    pos_orig_hector_[0] = tf.pose.position.x;
    pos_orig_hector_[1] = tf.pose.position.y;
    pos_orig_hector_[2] = 0;
    ori_orig_hector_[0] = tf.pose.orientation.w;
    ori_orig_hector_[1] = tf.pose.orientation.x;
    ori_orig_hector_[2] = tf.pose.orientation.y;
    ori_orig_hector_[3] = tf.pose.orientation.z;
    RCLCPP_INFO(this->get_logger(), "[%s]: Hector origin coordinates set - x: %f y: %f z: %f, w: %f, x: %f, y: %f, z: %f", this->get_name(),
                pos_orig_hector_[0], pos_orig_hector_[1], pos_orig_hector_[2], ori_orig_hector_[0], ori_orig_hector_[1], ori_orig_hector_[2],
                ori_orig_hector_[3]);
    hector_tf_setup_ = true;
  }
  /*//}*/

  // Initialize estimator timer
  if (!time_odometry_timer_set_) {
    time_odometry_timer_prev_ = std::chrono::high_resolution_clock::now();
    time_odometry_timer_set_  = true;
  }

  RCLCPP_INFO(this->get_logger(), "[%s]: Hector initalizied!", this->get_name());

  // Check if the estimator should be active
  if (!hector_active_) {
    hector_state_ = estimator_state_t::inactive;
    return;
  }
  hector_state_ = estimator_state_t::reliable;
}
//}

/* state_hector_reliable//{ */
// the following mutexes have to be locked by the calling function:
// hector_mutex_
void Odometry2::state_hector_reliable() {

  RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "Hector state: Reliable");

  std::scoped_lock lock(hector_raw_mutex_, hector_lat_estimator_mutex_, hector_lat_position_mutex_);

  // Check hector reliability
  if (!checkHectorReliability()) {
    hector_state_ = estimator_state_t::not_reliable;
    return;
  }

  // Add data into estimator, process the data

  // Calculate time since last estimators update
  std::chrono::time_point<std::chrono::high_resolution_clock> time_now = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double, std::milli>                   dt       = (time_now - time_odometry_timer_prev_) / 1000;

  if (dt.count() <= 0) {
    RCLCPP_WARN(this->get_logger(), "[%s]: odometry timer dt=%f is lower than zero, skipping estimator update.", this->get_name(), dt.count());
    return;
  }

  time_odometry_timer_prev_ = time_now;

  // Latitude estimator update and predict
  hector_lat_estimator_->doCorrection(hector_position_raw_[0], hector_position_raw_[1], LAT_HECTOR);
  hector_lat_estimator_->doPrediction(0.0, 0.0, dt.count());

  // Obtain lateral position from estimator
  double pos_x, pos_y, vel_x, vel_y;

  hector_lat_estimator_->getState(0, pos_x);
  hector_lat_estimator_->getState(1, pos_y);
  hector_lat_estimator_->getState(2, vel_x);
  hector_lat_estimator_->getState(3, vel_y);
  hector_lat_estimator_->getState(4, acc_x);
  hector_lat_estimator_->getState(5, acc_y);

  // Set variables for hector further usage
  hector_position_[0] = pos_x;
  hector_position_[1] = pos_y;
  hector_position_[2] = 0;  // TODO: The Z will be taken from garmin, but it can be directly used by pixhawk

  hector_velocity_[0] = vel_x;
  hector_velocity_[1] = vel_y;
  hector_velocity_[2] = 0;  // TODO: The Z will be taken from garmin, but it can be directly used by pixhawk
}
//}

/* state_hector_not_reliable//{ */
// the following mutexes have to be locked by the calling function:
// hector_mutex_
void odometry2::state_hector_not_reliable() {

  RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "Hector state: Not reliable.");

  // Check the last hector reset time
  std::chrono::duration<double> dt = std::chrono::system_clock::now() - hector_reset_called_time_;
  if (dt.count() > hector_reset_wait_) {
    RCLCPP_INFO(get_logger(), "[Odometry2]: Hector reset");
    hector_state_ = estimator_state_t::restart;
  } else {
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "[Odometry2]: Waiting for next hector reset availability. Call again in dt: %f",
                         hector_reset_wait_ - dt.count());
  }
}
//}

/* state_hector_restart//{ */
// the following mutexes have to be locked by the calling function:
// hector_mutex_
void odometry2::state_hector_restart() {

  RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "Hector state: Restart.");

  std::scoped_lock lock(hector_raw_mutex_, hector_lat_estimator_mutex_);

  // Reset hector
  auto future_response = reset_hector_client_->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
  if (f.wait_for(hector_reset_response_wait_) == std::future_status::timeout || f.get() == nullptr) {
    RCLCPP_ERROR(get_logger(), "[%s]: Could not reset hector, try again soon.", get_name());
    return;
  } else {
    RCLCPP_INFO(get_logger(), "[%s]: Hector node reset successfull.", get_name());
  }

  // Reset hector estimator and variables
  hector_lat_estimator_->setState(0, 0);
  hector_lat_estimator_->setState(1, 0);
  hector_lat_estimator_->setState(2, 0);
  hector_lat_estimator_->setState(3, 0);
  hector_lat_estimator_->setState(4, 0);
  hector_lat_estimator_->setState(5, 0);
  time_odometry_timer_set_ = false;

  hector_position_raw_[0]      = 0;
  hector_position_raw_[1]      = 0;
  hector_position_raw_prev_[0] = 0;
  hector_position_raw_prev_[1] = 0;

  c_hector_init_msgs_       = 0;
  hector_tf_setup_          = false;
  hector_reset_called_time_ = std::chrono::system_clock::now();
  getting_hector_           = false;

  RCLCPP_INFO(this->get_logger(), "[%s]: Hector restarted!", this->get_name());

  hector_state_ = estimator_state_t::init;
}
//}

// | --------------------- Hector methods --------------------- |

/* checkHectorReliability //{*/
// the following mutexes have to be locked by the calling function:
// hector_raw_mutex_
bool Odometry2::checkHectorReliability() {

  // Check hector message interval//{
  std::chrono::duration<double> dt = std::chrono::system_clock::now() - time_hector_last_msg_;

  if (dt.count() > hector_msg_interval_warn_) {
    RCLCPP_WARN(this->get_logger(), "[Odometry2]: Hector pose not received for %f seconds.", dt.count());
    if (dt.count() > hector_msg_interval_max_) {
      RCLCPP_WARN(this->get_logger(), "[Odometry2]: Hector pose not received for %f seconds. Not reliable.", dt.count());
      return false;
    }
  }
  /*//}*/

  // Detect position jump in consecutive hector poses//{
  if (std::pow(hector_position_raw_[0] - hector_position_raw_prev_[0], 2) > hector_max_position_jump_ ||
      std::pow(hector_position_raw_[1] - hector_position_raw_prev_[1], 2) > hector_max_position_jump_) {
    RCLCPP_WARN(this->get_logger(), "[Odometry2]: Jump detected in Hector Slam pose. orig_x: %f, orig_y: %f, x: %f, y: %f . Not reliable",
                hector_position_raw_prev_[0], hector_position_raw_prev_[1], hector_position_raw_[0], hector_position_raw_[1]);
    return false;
  }
  /*//}*/

  // No update in consecutive hector poses -> no features//{
  if (std::pow(hector_position_raw_[0] - hector_position_raw_prev_[0], 2) == 0 && std::pow(hector_position_raw_[1] - hector_position_raw_prev_[1], 2) == 0) {
    RCLCPP_WARN(this->get_logger(), "[Odometry2]: Hector does not have any features. orig_x: %f, orig_y: %f, x: %f, y: %f . Not reliable",
                hector_position_raw_prev_[0], hector_position_raw_prev_[1], hector_position_raw_[0], hector_position_raw_[1]);
    return false;
  }
  /*//}*/

  // Detect velocity jump in consecutive hector poses//{
  if (hector_velocity_[0] > hector_max_velocity_ || hector_velocity_[1] > hector_max_velocity_) {
    RCLCPP_WARN(this->get_logger(), "[Odometry2]: Hector velocity too large - x: %f, y: %f. Not reliable.", hector_velocity_[0], hector_velocity_[1]);
    return false;
  } /*//}*/

} /*//}*/

/* checkGpsReliability //{*/
// the following mutexes have to be locked by the calling function:
// gps_raw_mutex_
bool Odometry2::checkGpsReliability() {

  // Check gps message interval//{
  std::chrono::duration<double> dt = std::chrono::system_clock::now() - time_gps_last_msg_;

  if (dt.count() > gps_msg_interval_warn_) {
    RCLCPP_WARN(this->get_logger(), "[Odometry2]: GPS message not received for %f seconds.", dt.count());
    if (dt.count() > gps_msg_interval_max_) {
      RCLCPP_WARN(this->get_logger(), "[Odometry2]: GPS message not received for %f seconds. Not reliable.", dt.count());
      return false;
    }
  }
  /*//}*/

  /* Check gps quality//{*/
  if (msg->eph > gps_eph_max_) {
    if (c_gps_eph_err_++ >= gps_msg_err_) {  // GPS quality is lower for a specific number of consecutive messages
      RCLCPP_WARN(get_logger(), "[%s] GPS quality is too low! Not reliable", this->get_name());
      c_gps_eph_err_ = 0;
      return false;

    } else {
      RCLCPP_WARN(this->get_logger(), "[%s] GPS quality is too low! #%d/#%d EPH value: %f", this->get_name(), c_gps_eph_err_, gps_msg_err_, msg->eph);
      return true;
    }
  } else {
    c_gps_eph_err_ = 0;  // Reset the bad message counter
    return true;
  }
  /*//}*/

} /*//}*

// --------------------------------------------------------------
// |                           Publish                          |
// --------------------------------------------------------------

/* publishStaticTF //{ */
void Odometry2::publishStaticTF() {
  std::vector<geometry_msgs::msg::TransformStamped> v_transforms;

  geometry_msgs::msg::TransformStamped tf;
  tf2::Quaternion                      q;

  tf.header.stamp            = this->get_clock()->now();
  tf.header.frame_id         = frd_fcu_frame_;
  tf.child_frame_id          = fcu_frame_;
  tf.transform.translation.x = 0;
  tf.transform.translation.y = 0;
  tf.transform.translation.z = 0;
  q.setRPY(-M_PI, 0, 0);
  tf.transform.rotation.x = q.getX();
  tf.transform.rotation.y = q.getY();
  tf.transform.rotation.z = q.getZ();
  tf.transform.rotation.w = q.getW();
  v_transforms.push_back(tf);

  tf.header.frame_id         = world_frame_;
  tf.child_frame_id          = ned_origin_frame_;
  tf.transform.translation.x = 0.0;
  tf.transform.translation.y = 0.0;
  tf.transform.translation.z = 0.0;
  q.setRPY(M_PI, 0, M_PI_2);
  q                       = q.inverse();
  tf.transform.rotation.x = q.getX();
  tf.transform.rotation.y = q.getY();
  tf.transform.rotation.z = q.getZ();
  tf.transform.rotation.w = q.getW();
  v_transforms.push_back(tf);

  tf_world_to_ned_origin_frame_ = tf;

  static_tf_broadcaster_->sendTransform(v_transforms);

  publishing_static_tf_ = true;
}
//}

/* publishGpsTF //{ */
void Odometry2::publishGpsTF() {

  std::scoped_lock lock(px4_pose_mutex_);

  if (tf_broadcaster_ == nullptr) {
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this->shared_from_this());
  }
  geometry_msgs::msg::TransformStamped tf;
  tf2::Quaternion                      q;

  tf.header.stamp            = this->get_clock()->now();
  tf.header.frame_id         = ned_origin_frame_;
  tf.child_frame_id          = frd_fcu_frame_;
  tf.transform.translation.x = px4_position_[0];
  tf.transform.translation.y = px4_position_[1];
  tf.transform.translation.z = px4_position_[2];
  tf.transform.rotation.w    = px4_orientation_[0];
  tf.transform.rotation.x    = px4_orientation_[1];
  tf.transform.rotation.y    = px4_orientation_[2];
  tf.transform.rotation.z    = px4_orientation_[3];
  tf_broadcaster_->sendTransform(tf);
}
//}

/* publishGpsOdometry //{ */
void Odometry2::publishGpsOdometry() {

  std::scoped_lock lock(px4_pose_mutex_);

  // frd -> flu (enu) is rotation 180 degrees around x
  tf2::Quaternion q_orig, q_rot, q_new;

  q_orig.setW(px4_orientation_[0]);
  q_orig.setX(px4_orientation_[1]);
  q_orig.setY(px4_orientation_[2]);
  q_orig.setZ(px4_orientation_[3]);

  q_rot.setRPY(M_PI, 0, 0);
  q_new = q_orig * q_rot;
  q_new.normalize();

  geometry_msgs::msg::PoseStamped pose_ned, pose_enu;

  pose_ned.pose.position.x = px4_position_[0];
  pose_ned.pose.position.y = px4_position_[1];
  pose_ned.pose.position.z = px4_position_[2];
  tf2::convert(q_new, pose_ned.pose.orientation);

  tf2::doTransform(pose_ned, pose_enu, tf_local_origin_to_ned_origin_frame_);

  // Local odometry message
  nav_msgs::msg::Odometry odom_msg;

  odom_msg.header.stamp    = this->get_clock()->now();
  odom_msg.header.frame_id = local_origin_frame_;
  odom_msg.child_frame_id  = frd_fcu_frame_;
  odom_msg.pose.pose       = pose_enu.pose;

  // Heading message
  fog_msgs::msg::Heading heading_msg;

  heading_msg.header.stamp    = odom_msg.header.stamp;
  heading_msg.header.frame_id = local_origin_frame_;
  heading_msg.heading         = quat2heading(pose_enu.pose.orientation);

  // Publish messages
  local_odom_publisher_->publish(odom_msg);
  heading_publisher_->publish(heading_msg);
}
//}

/* publishHectorTF //{ */
void Odometry2::publishHectorTF() {

  std::scoped_lock lock(hector_lat_position_mutex_);

  if (tf_broadcaster_ == nullptr) {
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this->shared_from_this());
  }

  geometry_msgs::msg::TransformStamped tf, tf2;
  tf2::Quaternion                      q;

  // Publish tf of the hector_origin in the local_origin frame
  tf.header.stamp            = this->get_clock()->now();
  tf.header.frame_id         = local_origin_frame_;
  tf.child_frame_id          = hector_origin_frame_;
  tf.transform.translation.x = pos_orig_hector_[0];
  tf.transform.translation.y = pos_orig_hector_[1];
  tf.transform.translation.z = pos_orig_hector_[2];
  tf.transform.rotation.w    = ori_orig_hector_[0];
  tf.transform.rotation.x    = ori_orig_hector_[1];
  tf.transform.rotation.y    = ori_orig_hector_[2];
  tf.transform.rotation.z    = ori_orig_hector_[3];
  tf_broadcaster_->sendTransform(tf);

  // Publish tf of the current position in hector frame
  tf2.header.stamp            = this->get_clock()->now();
  tf2.header.frame_id         = hector_origin_frame_;
  tf2.child_frame_id          = hector_frame_;
  tf2.transform.translation.x = hector_position_[0];
  tf2.transform.translation.y = hector_position_[1];
  tf2.transform.translation.z = hector_position_[2];
  tf2.transform.rotation.w    = hector_orientation_[0];
  tf2.transform.rotation.x    = hector_orientation_[1];
  tf2.transform.rotation.y    = hector_orientation_[2];
  tf2.transform.rotation.z    = hector_orientation_[3];
  tf_broadcaster_->sendTransform(tf2);
}
//}

/* publishHectorOdometry //{ */
void Odometry2::publishHectorOdometry() {

  std::scoped_lock lock(px4_pose_mutex_, hector_lat_position_mutex_, timestamp_mutex_);

  if (!tf_buffer_->canTransform(ned_origin_frame_, hector_origin_frame_, rclcpp::Time(0))) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "[%s]: Missing hector origin frame - waiting for hector initialization.",
                         this->get_name());
    return;
  }

  auto transform_stamped = tf_buffer_->lookupTransform(ned_origin_frame_, hector_origin_frame_, rclcpp::Time(0));

  // Transform pose, orientation and velocity
  geometry_msgs::msg::PointStamped   hector_pos, hector_pos_tf;
  geometry_msgs::msg::Vector3Stamped hector_vel, hector_vel_tf;

  hector_pos.header.stamp    = this->get_clock()->now();
  hector_pos.header.frame_id = hector_origin_frame_;
  hector_pos.point.x         = hector_position_[0];
  hector_pos.point.y         = hector_position_[1];
  hector_pos.point.z         = hector_position_[2];
  tf2::doTransform(hector_pos, hector_pos_tf, transform_stamped);

  hector_vel.header.stamp    = this->get_clock()->now();
  hector_vel.header.frame_id = hector_origin_frame_;
  hector_vel.vector.x        = hector_velocity_[0];
  hector_vel.vector.y        = hector_velocity_[1];
  hector_vel.vector.z        = hector_velocity_[2];
  tf2::doTransform(hector_vel, hector_vel_tf, transform_stamped);


  // Visual odometry for px
  px4_msgs::msg::VehicleVisualOdometry visual_odometry;

  // Compensate time with timestamp message from px
  std::chrono::duration<long int, std::nano> diff = std::chrono::high_resolution_clock::now() - time_sync_time_;
  visual_odometry.timestamp                       = timestamp_ / 1000 + diff.count() / 1000;
  visual_odometry.timestamp_sample                = timestamp_raw_ / 1000 + diff.count() / 1000;

  visual_odometry.x = hector_pos_tf.point.x - pos_gps_offset_[0];
  visual_odometry.y = hector_pos_tf.point.y - pos_gps_offset_[1];
  visual_odometry.z = hector_pos_tf.point.z;

  visual_odometry.vx = hector_vel_tf.vector.x;
  visual_odometry.vy = hector_vel_tf.vector.y;
  visual_odometry.vz = hector_vel_tf.vector.z;

  visual_odometry.rollspeed  = NAN;
  visual_odometry.pitchspeed = NAN;
  visual_odometry.yawspeed   = NAN;

  visual_odometry.q[0] = px4_orientation_[0];
  visual_odometry.q[1] = px4_orientation_[1];
  visual_odometry.q[2] = px4_orientation_[2];
  visual_odometry.q[3] = px4_orientation_[3];

  std::fill(visual_odometry.q_offset.begin(), visual_odometry.q_offset.end(), NAN);
  std::fill(visual_odometry.pose_covariance.begin(), visual_odometry.pose_covariance.end(), NAN);
  std::fill(visual_odometry.velocity_covariance.begin(), visual_odometry.velocity_covariance.end(), NAN);

  // TODO:: STill needed? To align hector and gps
  //  current_visual_odometry_[0] = hector_pos_tf.point.x;
  //  current_visual_odometry_[1] = hector_pos_tf.point.y;

  // Odometry msg
  nav_msgs::msg::Odometry odom_msg;

  odom_msg.header.stamp            = this->get_clock()->now();
  odom_msg.header.frame_id         = hector_origin_frame_;
  odom_msg.child_frame_id          = hector_frame_;
  odom_msg.pose.pose.position.x    = hector_position_[0];
  odom_msg.pose.pose.position.y    = hector_position_[1];
  odom_msg.pose.pose.position.z    = hector_position_[2];
  odom_msg.pose.pose.orientation.w = hector_orientation_[0];
  odom_msg.pose.pose.orientation.x = hector_orientation_[1];
  odom_msg.pose.pose.orientation.y = hector_orientation_[2];
  odom_msg.pose.pose.orientation.z = hector_orientation_[3];
  odom_msg.twist.twist.linear.x    = hector_vel_tf.vector.x;
  odom_msg.twist.twist.linear.y    = hector_vel_tf.vector.y;
  odom_msg.twist.twist.linear.z    = hector_vel_tf.vector.z;

  // Publishing
  pixhawk_hector_publisher_->publish(odom_msg);
  visual_odom_publisher_->publish(visual_odometry);

  nav_msgs::msg::Odometry rviz_msg;

  // Publish hector in the correct RVIZ orientation visualization
  rviz_msg.header.stamp            = this->get_clock()->now();
  rviz_msg.header.frame_id         = local_origin_frame_;
  rviz_msg.child_frame_id          = hector_frame_;
  auto tf                          = transformBetween(fcu_frame_, local_origin_frame_);
  auto tf2                         = transformBetween(hector_frame_, local_origin_frame_);
  rviz_msg.pose.pose.position.x    = tf2.pose.position.x;
  rviz_msg.pose.pose.position.y    = tf2.pose.position.y;
  rviz_msg.pose.pose.position.z    = tf2.pose.position.z;
  rviz_msg.pose.pose.orientation.w = tf.pose.orientation.w;
  rviz_msg.pose.pose.orientation.x = tf.pose.orientation.x;
  rviz_msg.pose.pose.orientation.y = tf.pose.orientation.y;
  rviz_msg.pose.pose.orientation.z = tf.pose.orientation.z;

  local_hector_publisher_->publish(rviz_msg);
}
//}


// | ----------------------- Diagnostics ---------------------- |

/* publishOdometryDiagnostics //{ */
// the following mutexes have to be locked by the calling function:
// state_mutex_
void Odometry2::publishOdometryDiagnostics() {
  fog_msgs::msg::OdometryDiagnostics msg;

  msg.header.stamp    = get_clock()->now();
  msg.header.frame_id = world_frame_;

  msg.getting_pixhawk_odom      = getting_pixhawk_odom_;
  msg.getting_control_interface = getting_control_interface_;
  /* msg.vehicle_state = to_msg(vehicle_state_); */
  /* msg.mission_state = to_msg(mission_mgr_ ? mission_mgr_->state() : mission_state_t::finished); */

  /* msg.mission_id       = mission_mgr_ ? mission_mgr_->mission_id() : 0; */
  /* msg.mission_size     = mission_mgr_ ? mission_mgr_->mission_size() : 0; */
  /* msg.mission_waypoint = mission_mgr_ ? mission_mgr_->mission_waypoint() : 0; */

  /* msg.gps_origin_set       = gps_origin_set_; */
  /* msg.getting_odom         = getting_odom_; */
  /* msg.getting_control_mode = getting_control_mode_; */

  diagnostics_publisher_->publish(msg);
}
//}


// | -------------------------- Utils ------------------------- |

/* checkEstimatorReliability//{ */
// the following mutexes have to be locked by the calling function:
// state_mutex_
bool Odometry2::checkEstimatorReliability() {

  std::scoped_lock lock(gps_mutex_, hector_mutex_);

  odometry_state_t prev_state = odometry_state_;

  if (gps_state_ == estimator_state_t::reliable && hector_state_ == estimator_state_t::reliable) {
    odometry_state_ = odometry_state_t::gps;
  } else if (gps_state_ == estimator_state_t::reliable && hector_state_ != estimator_state_t::reliable) {
    odometry_state_ = odometry_state_t::gps;
  } else if (gps_state_ != estimator_state_t::reliable && hector_state_ == estimator_state_t::reliable) {
    odometry_state_ = odometry_state_t::hector;
  } else if (gps_state_ != estimator_state_t::reliable && hector_state_ != estimator_state_t::reliable) {
    odometry_state_ = odometry_state_t::missing_odometry;
  } else {
    odometry_state_ = odometry_state_t::invalid;
    assert(false);
    RCLCPP_ERROR(get_logger(), "This should not happen.");
  }

  // Check if odometry state has changed
  if (prev_state != odometry_state_) {
    odometry_update_ekf_ = true;
    return false;
  }
  return true;
}
//}

/* transformBetween //{ */
geometry_msgs::msg::PoseStamped Odometry2::transformBetween(std::string &frame_from, std::string &frame_to) {
  geometry_msgs::msg::PoseStamped pose_out;
  try {
    auto transform_stamped      = tf_buffer_->lookupTransform(frame_to, frame_from, rclcpp::Time(0));
    pose_out.pose.position.x    = transform_stamped.transform.translation.x;
    pose_out.pose.position.y    = transform_stamped.transform.translation.y;
    pose_out.pose.position.z    = transform_stamped.transform.translation.z;
    pose_out.pose.orientation.w = transform_stamped.transform.rotation.w;
    pose_out.pose.orientation.x = transform_stamped.transform.rotation.x;
    pose_out.pose.orientation.y = transform_stamped.transform.rotation.y;
    pose_out.pose.orientation.z = transform_stamped.transform.rotation.z;
  }
  catch (...) {
  }
  return pose_out;
}
//}

/*setInitialPx4Params//{*/
bool Odometry2::setInitialPx4Params() {

  if (!uploadPx4Parameters(fog_msgs::srv::SetPx4ParamInt::Request, px4_params_int_, set_px4_param_int_) ||
      !uploadPx4Parameters(fog_msgs::srv::SetPx4ParamFloat::Request, px4_params_float, set_px4_param_float_)) {
    return false;
  }
  return true;
}
/*//}*/

/* uploadPx4Parameters //{ */
template <typename T, typename U, typename V>
bool uploadPx4Parameters(const T &service_name, const U &param_array, const V &service_client) {
  for (const auto item : param_array) {
    auto request        = std::make_shared<service_name>();
    request->param_name = std::get<0>(item);
    request->value      = std::get<1>(item);
    RCLCPP_INFO(get_logger(), "[%s]: Setting %s, value: %f", get_name(), std::get<0>(item).c_str(), std::get<1>(item));
    auto future_response = service_client->async_send_request(request);

    // If parameter was not set, return false
    if (!checkPx4ParamSetOutput(future_response)) {
      return false;
    }
  }
  return true;
}
//}

/* checkPx4ParamSetOutput //{ */
template <typename T>
bool checkPx4ParamSetOutput(const std::shared_future<T> &f) {
  assert(f.valid());
  if (f.wait_for(px4_param_set_timeout_) == std::future_status::timeout || f.get() == nullptr) {
    RCLCPP_ERROR(get_logger(), "[%s]: Cannot set the parameter %s with message: %s", get_name(), f.get()->param_name.c_str(), f.get()->message.c_str());
    return false;
  } else {
    RCLCPP_INFO(get_logger(), "[%s]: Parameter %s has been set to value: %ld", get_name(), f.get()->param_name.c_str(), f.get()->value);
    return true;
  }
}
//}

/* new_cbk_grp() method //{ */
// just a util function that returns a new mutually exclusive callback group to shorten the call
rclcpp::CallbackGroup::SharedPtr Odometry::new_cbk_grp() {
  const rclcpp::CallbackGroup::SharedPtr new_group = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  callback_groups_.push_back(new_group);
  return new_group;
}
//}

/* //{ isValidType */
bool Odometry2::isValidType(const fog_msgs::msg::OdometryType &type) {

  if (type.type == fog_msgs::msg::OdometryType::GPSHECTOR || type.type == fog_msgs::msg::OdometryType::GPS ||
      type.type == fog_msgs::msg::OdometryType::HECTOR) {
    return true;
  }

  return false;
}

//}

}  // namespace odometry2


#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(odometry2::Odometry2)
