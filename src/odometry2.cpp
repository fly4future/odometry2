/* includes//{*/

#include <mutex>
#include <chrono>
#include <limits>
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
#include <fog_msgs/srv/change_odometry_source.hpp>
#include <fog_msgs/msg/control_interface_diagnostics.hpp>
#include <fog_msgs/msg/odometry_diagnostics.hpp>
#include <fog_msgs/msg/estimator_diagnostics.hpp>
#include <fog_msgs/msg/heading.hpp>

#include <fog_lib/scope_timer.h>
#include <fog_lib/params.h>
#include <fog_lib/median_filter.h>
#include <fog_lib/geometry/misc.h>

#include <px4_msgs/msg/timesync.hpp>
#include <px4_msgs/msg/vehicle_gps_position.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/vehicle_visual_odometry.hpp>
#include <px4_msgs/msg/distance_sensor.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>  // This has to be here otherwise you will get cryptic linker error about missing function 'getTimestamp'

#include <nav_msgs/msg/odometry.hpp>

#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "odometry/enums.h"
#include "odometry/lateral_estimator.h"
#include "odometry/altitude_estimator.h"

/*//}*/

typedef std::tuple<std::string, int>   px4_int;
typedef std::tuple<std::string, float> px4_float;

using namespace std::placeholders;
using namespace fog_lib;
using namespace fog_lib::geometry;

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
  odometry_state_t       last_set_parameters_ = odometry_state_t::init;
  std::vector<px4_int>   hector_px4_params_int_;
  std::vector<px4_float> hector_px4_params_float_;
  std::vector<px4_int>   gps_px4_params_int_;
  std::vector<px4_float> gps_px4_params_float_;
  std::atomic_bool       set_initial_px4_params_                = false;
  std::atomic_bool       getting_px4_utm_position_              = false;
  std::atomic_bool       getting_control_interface_diagnostics_ = false;
  std::atomic_bool       getting_pixhawk_odometry_              = false;
  int                    px4_param_set_timeout_                 = 0;

  float      px4_position_[3];
  float      px4_orientation_[4];
  std::mutex px4_pose_mutex_;

  int ekf_default_hgt_mode_ = 0;

  // | -------------------- PX home position -------------------- |
  double           px4_utm_position_[2];
  std::atomic_bool republish_static_tf_ = false;
  std::mutex       px4_utm_position_mutex_;

  // | --------------------- GPS parameters --------------------- |
  std::atomic_bool gps_active_  = false;
  std::atomic_bool getting_gps_ = false;
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

  std::chrono::time_point<std::chrono::system_clock> gps_last_msg_time_           = std::chrono::system_clock::now();  // Last received message from gps
  std::chrono::time_point<std::chrono::system_clock> gps_last_processed_msg_time_ = std::chrono::system_clock::now();

  // | -------------------- Hector parameters ------------------- |
  std::atomic_bool getting_hector_ = false;
  std::atomic_bool hector_active_  = false;
  std::mutex       hector_raw_mutex_;

  int    c_hector_init_msgs_         = 0;
  int    hector_num_init_msgs_       = 0;
  int    hector_reset_response_wait_ = 0;
  float  hector_msg_interval_max_    = 0.0;
  float  hector_msg_interval_warn_   = 0.0;
  double hector_hdg_previous_        = 0.0;
  float  hector_reset_wait_          = 0.0;
  float  hector_reliable_time_limit_ = 0.0;
  float  hector_max_position_jump_   = 0.0;
  float  hector_max_velocity_        = 0.0;

  float            hector_position_[3];
  float            hector_velocity_[3];
  float            hector_position_raw_prev_[2];
  float            hector_position_raw_[2];
  float            hector_orientation_[4];
  float            pos_orig_hector_[3];
  float            ori_orig_hector_[4];
  std::atomic_bool hector_tf_setup_ = false;
  std::atomic_bool published_hector = false;

  std::chrono::time_point<std::chrono::system_clock> hector_reset_called_time_ = std::chrono::system_clock::now();  // Last hector reset time
  std::chrono::time_point<std::chrono::system_clock> hector_last_msg_time_;
  std::chrono::time_point<std::chrono::system_clock> hector_reliable_time_;

  // | ------------------------- Garmin ------------------------- |
  std::atomic_bool getting_garmin_ = false;
  std::mutex       garmin_mutex_;
  double           garmin_measurement_;

  int                           garmin_num_init_msgs_       = 0;
  int                           garmin_num_avg_offset_msgs_ = 0;
  int                           c_garmin_init_msgs_         = 0;
  float                         garmin_offset_              = 0;
  std::vector<float>            garmin_init_values_;
  std::unique_ptr<MedianFilter> alt_mf_garmin_;
  double                        _garmin_min_valid_alt_;
  double                        _garmin_max_valid_alt_;

  // | -------------------- PX Vision Message ------------------- |
  unsigned long long                                          timestamp_;
  std::int64_t                                                timestamp_raw_;
  std::chrono::time_point<std::chrono::high_resolution_clock> time_sync_time_;
  std::mutex                                                  timestamp_mutex_;

  // | -------------------- Lateral estimator ------------------- |

  std::chrono::time_point<std::chrono::high_resolution_clock> time_odometry_timer_prev_;
  std::atomic_bool                                            time_odometry_timer_set_ = false;
  std::mutex                                                  hector_lat_estimator_mutex_, hector_lat_position_mutex_;

  std::shared_ptr<LateralEstimator> hector_lat_estimator_;
  std::vector<lat_R_t>              R_lat_vec_;

  // | ------------------- Altitude estimator ------------------- |

  std::shared_ptr<AltitudeEstimator> garmin_alt_estimator_;
  std::vector<alt_R_t>               R_alt_vec_;

  // | --------------------- Callback groups -------------------- |
  // a shared pointer to each callback group has to be saved or the callbacks will never get called
  std::vector<rclcpp::CallbackGroup::SharedPtr> callback_groups_;
  // new callback groups have to be initialized using this function to be saved into callback_groups_
  rclcpp::CallbackGroup::SharedPtr new_cbk_grp();

  // | ----------------------- Publishers ----------------------- |
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr              local_odom_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr              local_hector_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr              pixhawk_hector_publisher_;  // Output hector message in odometry format
  rclcpp::Publisher<fog_msgs::msg::OdometryDiagnostics>::SharedPtr   odometry_diagnostics_publisher_;
  rclcpp::Publisher<fog_msgs::msg::EstimatorDiagnostics>::SharedPtr  gps_diagnostics_publisher_;
  rclcpp::Publisher<fog_msgs::msg::EstimatorDiagnostics>::SharedPtr  hector_diagnostics_publisher_;
  rclcpp::Publisher<px4_msgs::msg::VehicleVisualOdometry>::SharedPtr hector_odometry_publisher_;
  rclcpp::Publisher<fog_msgs::msg::Heading>::SharedPtr               heading_publisher_;

  // | ----------------------- Subscribers ---------------------- |
  rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr             pixhawk_odom_subscriber_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr            hector_pose_subscriber_;
  rclcpp::Subscription<px4_msgs::msg::Timesync>::SharedPtr                    timesync_subscriber_;
  rclcpp::Subscription<px4_msgs::msg::VehicleGpsPosition>::SharedPtr          gps_subscriber_;
  rclcpp::Subscription<fog_msgs::msg::ControlInterfaceDiagnostics>::SharedPtr control_interface_diagnostics_subscriber_;
  rclcpp::Subscription<px4_msgs::msg::DistanceSensor>::SharedPtr              garmin_subscriber_;

  // | -------------------- Service providers ------------------- |
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr              reset_hector_service_;
  rclcpp::Service<fog_msgs::srv::ChangeOdometrySource>::SharedPtr change_odometry_source_;

  // | --------------------- Service clients -------------------- |
  rclcpp::Client<fog_msgs::srv::SetPx4ParamInt>::SharedPtr   set_px4_param_int_;
  rclcpp::Client<fog_msgs::srv::SetPx4ParamFloat>::SharedPtr set_px4_param_float_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr          reset_hector_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr          land_client_;

  // | ------------------ Subscriber callbacks ------------------ |
  void pixhawkOdomCallback(const px4_msgs::msg::VehicleOdometry::UniquePtr msg);
  void hectorPoseCallback(const geometry_msgs::msg::PoseStamped::UniquePtr msg);
  void timesyncCallback(const px4_msgs::msg::Timesync::UniquePtr msg);
  void gpsCallback(const px4_msgs::msg::VehicleGpsPosition::UniquePtr msg);
  void ControlInterfaceDiagnosticsCallback(const fog_msgs::msg::ControlInterfaceDiagnostics::UniquePtr msg);
  void garminCallback(const px4_msgs::msg::DistanceSensor::UniquePtr msg);

  // | -------------------- Service callbacks ------------------- |
  bool resetHectorCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request, std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  bool changeOdometryCallback(const std::shared_ptr<fog_msgs::srv::ChangeOdometrySource::Request> request,
                              std::shared_ptr<fog_msgs::srv::ChangeOdometrySource::Response>      response);

  // | ----------------------- Publishing ----------------------- |
  void publishStaticTF();
  void publishGpsTF();
  void publishHectorTF();

  void publishGpsOdometry();
  void publishHectorOdometry();

  // --------------------------------------------------------------
  // |                     Internal functions                     |
  // --------------------------------------------------------------
  bool checkHectorReliability();
  bool checkGpsReliability();
  bool isValidType(const fog_msgs::msg::OdometryType& type);
  bool isValidGate(const double& value, const double& min_value, const double& max_value, const std::string& value_name);

  // --------------------------------------------------------------
  // |                          Routines                          |
  // --------------------------------------------------------------

  // | ------------------ Odometry node routine ----------------- |
  std::recursive_mutex odometry_mutex_;
  odometry_state_t     odometry_state_  = odometry_state_t::not_connected;
  odometry_state_t     switching_state_ = odometry_state_t::init;  // State indicating the targeting switch state
  double               odometry_loop_rate_;

  rclcpp::TimerBase::SharedPtr odometry_timer_;
  rclcpp::TimerBase::SharedPtr odometry_diagnostics_timer_;
  rclcpp::TimerBase::SharedPtr odometry_publisher_timer_;

  void odometryRoutine(void);
  void odometryDiagnosticsRoutine(void);
  void odometryPublisherRoutine(void);

  void update_odometry_state();
  void state_odometry_not_connected();
  void state_odometry_init();
  void state_odometry_switching();
  void state_odometry_gps();
  void state_odometry_hector();
  void state_odometry_missing_odometry();
  void state_odometry_waiting_odometry();

  void publishOdometryDiagnostics();
  void publishTF();
  void publishOdometry();

  // | ------------------ GPS estimator routine ----------------- |
  std::recursive_mutex gps_mutex_;
  estimator_state_t    gps_state_ = estimator_state_t::init;

  rclcpp::TimerBase::SharedPtr gps_timer_;
  rclcpp::TimerBase::SharedPtr gps_diagnostics_timer_;

  void gpsRoutine(void);
  void gpsDiagnosticsRoutine(void);

  double gps_loop_rate_;
  void   update_gps_state();

  void state_gps_init();
  void state_gps_reliable();
  void state_gps_not_reliable();
  void state_gps_inactive();
  void state_gps_restart();
  void state_gps_restarting();

  void publishGpsDiagnostics();

  // | ------------------ Hector estimator routine ----------------- |
  std::recursive_mutex hector_mutex_;
  estimator_state_t    hector_state_ = estimator_state_t::init;

  rclcpp::TimerBase::SharedPtr hector_timer_;
  rclcpp::TimerBase::SharedPtr hector_diagnostics_timer_;

  void hectorRoutine(void);
  void hectorDiagnosticsRoutine(void);

  double hector_loop_rate_;
  void   update_hector_state();

  void state_hector_init();
  void state_hector_reliable();
  void state_hector_not_reliable();
  void state_hector_restart();
  void state_hector_restarting();
  void state_hector_inactive();

  void publishHectorDiagnostics();
  void updateHectorEstimator();

  // | -------------------- Routine handling -------------------- |
  rclcpp::CallbackGroup::SharedPtr callback_group_;

  // | --------------------- Utils function --------------------- |
  bool setPx4Params(const std::vector<px4_int>& params_int, const std::vector<px4_float>& params_float);
  bool updateEkfParameters();
  void checkEstimatorReliability();

  geometry_msgs::msg::PoseStamped transformBetween(std::string& frame_from, std::string& frame_to);

  template <typename T, typename U, typename V>
  bool uploadPx4Parameters(const std::shared_ptr<T>& service_name, const std::vector<U>& param_array, const V& service_client);

  template <typename T>
  bool checkPx4ParamSetOutput(const std::shared_future<T> f);
};
//}

/* constructor //{ */
Odometry2::Odometry2(rclcpp::NodeOptions options) : Node("odometry2", options) {

  RCLCPP_INFO(get_logger(), "Node initializing");

  try {
    uav_name_ = std::string(std::getenv("DRONE_DEVICE_ID"));
  }
  catch (...) {
    RCLCPP_WARN(get_logger(), "Environment variable DRONE_DEVICE_ID was not defined!");
  }
  RCLCPP_INFO(get_logger(), "UAV name is: '%s'", uav_name_.c_str());

  /* parse general params from config file //{ */
  RCLCPP_INFO(get_logger(), "-------------- Loading parameters --------------");
  bool loaded_successfully = true;
  bool temp;

  loaded_successfully &= parse_param("odometry_loop_rate", odometry_loop_rate_, *this);
  loaded_successfully &= parse_param("gps_loop_rate", gps_loop_rate_, *this);
  loaded_successfully &= parse_param("hector_loop_rate", hector_loop_rate_, *this);

  loaded_successfully &= parse_param("gps.active", temp, *this);
  gps_active_.store(temp);
  loaded_successfully &= parse_param("gps.eph_max", gps_eph_max_, *this);
  loaded_successfully &= parse_param("gps.msg_good", gps_msg_good_, *this);
  loaded_successfully &= parse_param("gps.msg_err", gps_msg_err_, *this);
  loaded_successfully &= parse_param("gps.num_init_msgs", gps_num_init_msgs_, *this);
  loaded_successfully &= parse_param("gps.msg_interval_max", gps_msg_interval_max_, *this);
  loaded_successfully &= parse_param("gps.msg_interval_warn", gps_msg_interval_warn_, *this);

  loaded_successfully &= parse_param("hector.active", temp, *this);
  hector_active_.store(temp);
  loaded_successfully &= parse_param("hector.reset_wait", hector_reset_wait_, *this);
  loaded_successfully &= parse_param("hector.reliable_time_limit", hector_reliable_time_limit_, *this);
  loaded_successfully &= parse_param("hector.reset_response_wait", hector_reset_response_wait_, *this);
  loaded_successfully &= parse_param("hector.num_init_msgs", hector_num_init_msgs_, *this);
  loaded_successfully &= parse_param("hector.msg_interval_max", hector_msg_interval_max_, *this);
  loaded_successfully &= parse_param("hector.msg_interval_warn", hector_msg_interval_warn_, *this);
  loaded_successfully &= parse_param("hector.max_position_jump", hector_max_position_jump_, *this);
  loaded_successfully &= parse_param("hector.max_velocity", hector_max_velocity_, *this);

  loaded_successfully &= parse_param("altitude.avg_offset_msgs", garmin_num_avg_offset_msgs_, *this);
  loaded_successfully &= parse_param("altitude.num_init_msgs", garmin_num_init_msgs_, *this);

  loaded_successfully &= parse_param("px4.param_set_timeout", px4_param_set_timeout_, *this);

  int   param_int;
  float param_float;

  /*Hector px params loading//{*/
  loaded_successfully &= parse_param("px4.hector.EKF2_AID_MASK", param_int, *this);
  hector_px4_params_int_.push_back(px4_int("EKF2_AID_MASK", param_int));
  loaded_successfully &= parse_param("px4.hector.EKF2_EV_NOISE_MD", param_int, *this);
  hector_px4_params_int_.push_back(px4_int("EKF2_EV_NOISE_MD", param_int));
  loaded_successfully &= parse_param("px4.hector.EKF2_RNG_AID", param_int, *this);
  hector_px4_params_int_.push_back(px4_int("EKF2_RNG_AID", param_int));
  loaded_successfully &= parse_param("px4.hector.EKF2_HGT_MODE", param_int, *this);
  hector_px4_params_int_.push_back(px4_int("EKF2_HGT_MODE", param_int));

  loaded_successfully &= parse_param("px4.hector.EKF2_EV_DELAY", param_float, *this);
  hector_px4_params_float_.push_back(px4_float("EKF2_EV_DELAY", param_float));
  loaded_successfully &= parse_param("px4.hector.EKF2_EVP_NOISE", param_float, *this);
  hector_px4_params_float_.push_back(px4_float("EKF2_EVP_NOISE", param_float));
  loaded_successfully &= parse_param("px4.hector.EKF2_EVV_NOISE", param_float, *this);
  hector_px4_params_float_.push_back(px4_float("EKF2_EVV_NOISE", param_float));
  loaded_successfully &= parse_param("px4.hector.EKF2_RNG_A_HMAX", param_float, *this);
  hector_px4_params_float_.push_back(px4_float("EKF2_RNG_A_HMAX", param_float));
  loaded_successfully &= parse_param("px4.hector.MPC_XY_CRUISE", param_float, *this);
  hector_px4_params_float_.push_back(px4_float("MPC_XY_CRUISE", param_float));
  loaded_successfully &= parse_param("px4.hector.MC_YAWRATE_MAX", param_float, *this);
  hector_px4_params_float_.push_back(px4_float("MC_YAWRATE_MAX", param_float));
  loaded_successfully &= parse_param("px4.hector.MPC_ACC_HOR", param_float, *this);
  hector_px4_params_float_.push_back(px4_float("MPC_ACC_HOR", param_float));
  loaded_successfully &= parse_param("px4.hector.MPC_ACC_HOR_MAX", param_float, *this);
  hector_px4_params_float_.push_back(px4_float("MPC_ACC_HOR_MAX", param_float));
  loaded_successfully &= parse_param("px4.hector.MPC_JERK_AUTO", param_float, *this);
  hector_px4_params_float_.push_back(px4_float("MPC_JERK_AUTO", param_float));
  loaded_successfully &= parse_param("px4.hector.MPC_JERK_MAX", param_float, *this);
  hector_px4_params_float_.push_back(px4_float("MPC_JERK_MAX", param_float));
  loaded_successfully &= parse_param("px4.hector.MPC_ACC_DOWN_MAX", param_float, *this);
  hector_px4_params_float_.push_back(px4_float("MPC_ACC_DOWN_MAX", param_float));
  loaded_successfully &= parse_param("px4.hector.MPC_ACC_UP_MAX", param_float, *this);
  hector_px4_params_float_.push_back(px4_float("MPC_ACC_UP_MAX", param_float));
  /*//}*/

  /*GPS px params loading//{*/
  loaded_successfully &= parse_param("px4.gps.EKF2_AID_MASK", param_int, *this);
  gps_px4_params_int_.push_back(px4_int("EKF2_AID_MASK", param_int));
  loaded_successfully &= parse_param("px4.gps.EKF2_RNG_AID", param_int, *this);
  gps_px4_params_int_.push_back(px4_int("EKF2_RNG_AID", param_int));
  loaded_successfully &= parse_param("px4.gps.EKF2_HGT_MODE", param_int, *this);
  gps_px4_params_int_.push_back(px4_int("EKF2_HGT_MODE", param_int));

  loaded_successfully &= parse_param("px4.gps.EKF2_RNG_A_HMAX", param_float, *this);
  gps_px4_params_float_.push_back(px4_float("EKF2_RNG_A_HMAX", param_float));
  loaded_successfully &= parse_param("px4.gps.MPC_XY_CRUISE", param_float, *this);
  gps_px4_params_float_.push_back(px4_float("MPC_XY_CRUISE", param_float));
  loaded_successfully &= parse_param("px4.gps.MC_YAWRATE_MAX", param_float, *this);
  gps_px4_params_float_.push_back(px4_float("MC_YAWRATE_MAX", param_float));
  loaded_successfully &= parse_param("px4.gps.MPC_ACC_HOR", param_float, *this);
  gps_px4_params_float_.push_back(px4_float("MPC_ACC_HOR", param_float));
  loaded_successfully &= parse_param("px4.gps.MPC_ACC_HOR_MAX", param_float, *this);
  gps_px4_params_float_.push_back(px4_float("MPC_ACC_HOR_MAX", param_float));
  loaded_successfully &= parse_param("px4.gps.MPC_JERK_AUTO", param_float, *this);
  gps_px4_params_float_.push_back(px4_float("MPC_JERK_AUTO", param_float));
  loaded_successfully &= parse_param("px4.gps.MPC_JERK_MAX", param_float, *this);
  gps_px4_params_float_.push_back(px4_float("MPC_JERK_MAX", param_float));
  loaded_successfully &= parse_param("px4.gps.MPC_ACC_DOWN_MAX", param_float, *this);
  gps_px4_params_float_.push_back(px4_float("MPC_ACC_DOWN_MAX", param_float));
  loaded_successfully &= parse_param("px4.gps.MPC_ACC_UP_MAX", param_float, *this);
  gps_px4_params_float_.push_back(px4_float("MPC_ACC_UP_MAX", param_float));
  /*//}*/
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

  /* lateral measurement covariances (R matrices) //{ */

  lat_R_t R_lat;
  double  R_lat_tmp;
  loaded_successfully &= parse_param("lateral.measurement_covariance.hector", R_lat_tmp, *this);
  R_lat = R_lat.Identity() * R_lat_tmp;
  R_lat_vec_.push_back(R_lat);

  //}

  /* lateral process covariance (Q matrix) //{ */

  lat_Q_t Q_lat = lat_Q_t::Identity();
  double  lat_pos, lat_vel, lat_acc;
  loaded_successfully &= parse_param("lateral.process_covariance.pos", lat_pos, *this);
  Q_lat(LAT_POS_X, LAT_POS_X) *= lat_pos;
  Q_lat(LAT_POS_Y, LAT_POS_Y) *= lat_pos;
  loaded_successfully &= parse_param("lateral.process_covariance.vel", lat_vel, *this);
  Q_lat(LAT_VEL_X, LAT_VEL_X) *= lat_vel;
  Q_lat(LAT_VEL_Y, LAT_VEL_Y) *= lat_vel;
  loaded_successfully &= parse_param("lateral.process_covariance.acc", lat_acc, *this);
  Q_lat(LAT_ACC_X, LAT_ACC_X) *= lat_acc;
  Q_lat(LAT_ACC_Y, LAT_ACC_Y) *= lat_acc;

  //}

  hector_lat_estimator_ = std::make_shared<LateralEstimator>("hector", Q_lat, R_lat_vec_);

  // | ------------------- Altitude estimator ------------------- |

  /* altitude median filters //{ */

  double buffer_size, max_valid, min_valid, max_diff;

  // We want to gate the measurements before median filtering to prevent the median becoming an invalid value
  min_valid = -1000.0;
  max_valid = 1000.0;

  // Garmin
  loaded_successfully &= parse_param("altitude.median_filter.garmin.buffer_size", buffer_size, *this);
  loaded_successfully &= parse_param("altitude.median_filter.garmin.max_diff", max_diff, *this);

  alt_mf_garmin_ = std::make_unique<MedianFilter>(buffer_size, max_valid, min_valid, max_diff, *this);

  //}

  /* altitude measurement min and max value gates //{ */

  loaded_successfully &= parse_param("altitude.gate.garmin.min", _garmin_min_valid_alt_, *this);
  loaded_successfully &= parse_param("altitude.gate.garmin.max", _garmin_max_valid_alt_, *this);

  //}

  /* altitude measurement covariances (R matrices) //{ */

  alt_R_t R_alt;
  double  R_alt_tmp;
  loaded_successfully &= parse_param("altitude.measurement_covariance.garmin", R_alt_tmp, *this);
  R_alt = R_alt.Identity() * R_alt_tmp;
  R_alt_vec_.push_back(R_alt);
  loaded_successfully &= parse_param("altitude.measurement_covariance.baro", R_alt_tmp, *this);
  R_alt = R_alt.Identity() * R_alt_tmp;
  R_alt_vec_.push_back(R_alt);

  //}

  /* altitude process covariance (Q matrix) //{ */

  alt_Q_t Q_alt = alt_Q_t::Identity();
  double  alt_pos, alt_vel, alt_acc;
  loaded_successfully &= parse_param("altitude.process_covariance.pos", alt_pos, *this);
  Q_alt(STATE_POS, STATE_POS) *= alt_pos;
  loaded_successfully &= parse_param("altitude.process_covariance.vel", alt_vel, *this);
  Q_alt(STATE_VEL, STATE_VEL) *= alt_vel;
  loaded_successfully &= parse_param("altitude.process_covariance.acc", alt_acc, *this);
  Q_alt(STATE_ACC, STATE_ACC) *= alt_acc;

  //}

  garmin_alt_estimator_ = std::make_shared<AltitudeEstimator>("garmin", Q_alt, R_alt_vec_);

  // Check if all parameters were loaded correctly
  if (!loaded_successfully) {
    const std::string str = "Could not load all non-optional parameters. Shutting down.";
    RCLCPP_ERROR(get_logger(), "%s", str.c_str());
    rclcpp::shutdown();
    return;
  }

  // | ----------------------- Publishers ----------------------- |
  rclcpp::QoS qos(rclcpp::KeepLast(3));
  local_odom_publisher_           = this->create_publisher<nav_msgs::msg::Odometry>("~/local_odom_out", qos);
  local_hector_publisher_         = this->create_publisher<nav_msgs::msg::Odometry>("~/local_hector_out", qos);
  pixhawk_hector_publisher_       = this->create_publisher<nav_msgs::msg::Odometry>("~/pixhawk_hector_out", qos);
  odometry_diagnostics_publisher_ = this->create_publisher<fog_msgs::msg::OdometryDiagnostics>("~/odometry_diagnostics_out", qos);
  gps_diagnostics_publisher_      = this->create_publisher<fog_msgs::msg::EstimatorDiagnostics>("~/gps_diagnostics_out", qos);
  hector_diagnostics_publisher_   = this->create_publisher<fog_msgs::msg::EstimatorDiagnostics>("~/hector_diagnostics_out", qos);
  hector_odometry_publisher_      = this->create_publisher<px4_msgs::msg::VehicleVisualOdometry>("~/hector_odometry_out", qos);
  heading_publisher_              = this->create_publisher<fog_msgs::msg::Heading>("~/heading_out", 10);

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
  subopts.callback_group  = new_cbk_grp();
  garmin_subscriber_      = this->create_subscription<px4_msgs::msg::DistanceSensor>("~/garmin_in", rclcpp::SystemDefaultsQoS(),
                                                                                std::bind(&Odometry2::garminCallback, this, _1), subopts);

  // | -------------------- Service clients  -------------------- |
  set_px4_param_int_   = this->create_client<fog_msgs::srv::SetPx4ParamInt>("~/set_px4_param_int");
  set_px4_param_float_ = this->create_client<fog_msgs::srv::SetPx4ParamFloat>("~/set_px4_param_float");
  reset_hector_client_ = this->create_client<std_srvs::srv::Trigger>("~/reset_hector_service_out");
  land_client_         = this->create_client<std_srvs::srv::Trigger>("~/land_service_out");

  // | -------------------- Service handlers -------------------- |
  const auto qos_profile  = qos.get_rmw_qos_profile();
  const auto srv_grp_ptr  = new_cbk_grp();
  reset_hector_service_   = this->create_service<std_srvs::srv::Trigger>("~/reset_hector_service_in", std::bind(&Odometry2::resetHectorCallback, this, _1, _2),
                                                                       qos_profile, srv_grp_ptr);
  change_odometry_source_ = this->create_service<fog_msgs::srv::ChangeOdometrySource>(
      "~/change_odometry_source_in", std::bind(&Odometry2::changeOdometryCallback, this, _1, _2), qos_profile, srv_grp_ptr);

  // --------------------------------------------------------------
  // |                          Routines                          |
  // --------------------------------------------------------------

  // | -------------------- Odometry routine -------------------- |

  odometry_timer_ =
      this->create_wall_timer(std::chrono::duration<double>(1.0 / odometry_loop_rate_), std::bind(&Odometry2::odometryRoutine, this), new_cbk_grp());
  odometry_diagnostics_timer_ =
      this->create_wall_timer(std::chrono::duration<double>(1.0 / odometry_loop_rate_), std::bind(&Odometry2::odometryDiagnosticsRoutine, this), new_cbk_grp());
  odometry_publisher_timer_ =
      this->create_wall_timer(std::chrono::duration<double>(1.0 / odometry_loop_rate_), std::bind(&Odometry2::odometryPublisherRoutine, this), new_cbk_grp());

  // | -------------------- GPS routine -------------------- |

  gps_timer_ = this->create_wall_timer(std::chrono::duration<double>(1.0 / gps_loop_rate_), std::bind(&Odometry2::gpsRoutine, this), new_cbk_grp());
  gps_diagnostics_timer_ =
      this->create_wall_timer(std::chrono::duration<double>(1.0 / gps_loop_rate_), std::bind(&Odometry2::gpsDiagnosticsRoutine, this), new_cbk_grp());

  // | -------------------- Hector routine -------------------- |

  hector_timer_ = this->create_wall_timer(std::chrono::duration<double>(1.0 / hector_loop_rate_), std::bind(&Odometry2::hectorRoutine, this), new_cbk_grp());
  hector_diagnostics_timer_ =
      this->create_wall_timer(std::chrono::duration<double>(1.0 / hector_loop_rate_), std::bind(&Odometry2::hectorDiagnosticsRoutine, this), new_cbk_grp());

  tf_broadcaster_        = nullptr;
  static_tf_broadcaster_ = nullptr;

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_buffer_->setUsingDedicatedThread(true);
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this, false);

  is_initialized_ = true;
  RCLCPP_INFO(get_logger(), "Node initialized");
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

  std::scoped_lock lock(timestamp_mutex_);

  timestamp_      = msg->ts1;
  timestamp_raw_  = msg->tc1;
  time_sync_time_ = std::chrono::high_resolution_clock::now();
}
//}

/* hectorPoseCallback //{ */
void Odometry2::hectorPoseCallback(const geometry_msgs::msg::PoseStamped::UniquePtr msg) {

  // If not initialized
  if (!is_initialized_) {
    return;
  }

  RCLCPP_INFO_ONCE(get_logger(), "Hector: Getting hector poses!");

  std::scoped_lock lock(hector_raw_mutex_);

  hector_last_msg_time_ = std::chrono::system_clock::now();

  /* Infinite value check//{*/
  if (!std::isfinite(msg->pose.position.x)) {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000, "Not finite value detected in variable \"pose.position.x\" (hectorcallback) !!!");
    return;
  }

  if (!std::isfinite(msg->pose.position.y)) {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000, "Not finite value detected in variable \"pose.position.y\" (hectorCallback) !!!");
    return;
  } /*//}*/

  /*Hector convergence//{*/
  // Wait for hector to converge
  if (c_hector_init_msgs_++ < hector_num_init_msgs_) {
    /* RCLCPP_INFO(get_logger(), "Hector pose #%d - x: %f y: %f", c_hector_init_msgs_, msg->pose.position.x, msg->pose.position.y); */
    hector_position_raw_[0] = msg->pose.position.x;
    hector_position_raw_[1] = msg->pose.position.y;
    return;
  } /*//}*/

  // Update previous hector position for the reliability check
  hector_position_raw_prev_[0] = hector_position_raw_[0];
  hector_position_raw_prev_[1] = hector_position_raw_[1];

  // Save current hector position
  hector_position_raw_[0] = msg->pose.position.x;
  hector_position_raw_[1] = msg->pose.position.y;

  getting_hector_ = true;
}
//}

/* garminCallback //{ */
void Odometry2::garminCallback(const px4_msgs::msg::DistanceSensor::UniquePtr msg) {

  if (!is_initialized_) {
    return;
  }

  std::scoped_lock lock(garmin_mutex_);

  RCLCPP_INFO_ONCE(get_logger(), "Getting garmin!");

  double measurement = msg->current_distance;

  // Check finite measurement
  if (!std::isfinite(measurement)) {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000, "Not finite value detected in variable \"measurement\" (garminCallback) !!!");
    return;
  }

  // Check if the value is between allowed limits
  if (!isValidGate(measurement, _garmin_min_valid_alt_, _garmin_max_valid_alt_, "garmin range")) {
    return;
  }

  // Skip number of messages before system initialization
  if (c_garmin_init_msgs_++ < garmin_num_init_msgs_) {
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 100, "Garmin measurement #%d - z: %f", c_garmin_init_msgs_, measurement);
    return;
  }

  // Gather offset amount of msgs
  if (garmin_init_values_.size() < garmin_num_avg_offset_msgs_) {
    garmin_init_values_.push_back(measurement);
    return;
  }

  // Set the garmin offset
  if (garmin_offset_ == 0) {
    garmin_offset_ = std::reduce(garmin_init_values_.begin(), garmin_init_values_.end()) / garmin_init_values_.size();
    RCLCPP_WARN(get_logger(), "Garmin offset - z: %f", garmin_offset_);
  }

  // do not fuse garmin measurements when a height jump is detected - most likely the UAV is flying above an obstacle
  if (!alt_mf_garmin_->isValid(measurement, *this)) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "[%s]: Garmin measurement %f declined by median filter.", this->get_name(), measurement);
    return;
  }

  RCLCPP_INFO_ONCE(get_logger(), "Getting Garmin altitude corrections");
  garmin_measurement_ = measurement;
  getting_garmin_     = true;
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

  getting_pixhawk_odometry_ = true;
  RCLCPP_INFO_ONCE(get_logger(), "Getting pixhawk odometry!");
}
//}

/* gpsCallback //{ */
void Odometry2::gpsCallback(const px4_msgs::msg::VehicleGpsPosition::UniquePtr msg) {

  if (!is_initialized_) {
    return;
  }

  RCLCPP_INFO_ONCE(get_logger(), "GPS: Getting Vehicle GPS!");

  std::scoped_lock lock(gps_raw_mutex_);

  gps_last_msg_time_ = std::chrono::system_clock::now();

  // Waiting for gps convergence after bootup
  if (c_gps_init_msgs_++ < gps_num_init_msgs_) {
    RCLCPP_INFO(get_logger(), "GPS pose #%d - lat: %d lon: %d", c_gps_init_msgs_, msg->lat, msg->lon);
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
  RCLCPP_INFO_ONCE(get_logger(), "Getting control diagnostics!");
}
//}

// --------------------------------------------------------------
// |                      Service Callbacks                     |
// --------------------------------------------------------------

/* changeOdometryCallback //{ */
bool Odometry2::changeOdometryCallback(const std::shared_ptr<fog_msgs::srv::ChangeOdometrySource::Request>  request,
                                       const std::shared_ptr<fog_msgs::srv::ChangeOdometrySource::Response> response) {

  if (!is_initialized_) {
    return false;
  }

  // Check whether a valid type was requested
  if (!isValidType(request->odometry_type)) {
    RCLCPP_ERROR(get_logger(), "Value %d is not a valid odometry type", request->odometry_type.type);
    response->success = false;
    response->message = "Not a valid odometry type";
    return true;
  }

  std::scoped_lock lock(gps_mutex_, hector_mutex_);

  // Change the estimator
  if (request->odometry_type.type == 1) {
    if (gps_state_ != estimator_state_t::reliable) {
      gps_state_ = estimator_state_t::not_reliable;
    }
    if (hector_state_ != estimator_state_t::reliable) {
      hector_state_         = estimator_state_t::not_reliable;
      hector_reliable_time_ = std::chrono::system_clock::now();
    }
    RCLCPP_WARN(get_logger(), "Changing odometry to automatic GPS or HECTOR");
  } else if (request->odometry_type.type == 2) {
    if (gps_state_ != estimator_state_t::reliable) {
      response->success = false;
      response->message = "Cannot change only to GPS. GPS is not reliable. Current GPS state is " + to_string(gps_state_);
      return true;
    }
    hector_state_ = estimator_state_t::inactive;
    RCLCPP_WARN(get_logger(), "Changing odometry to GPS only");
  } else if (request->odometry_type.type == 3) {
    if (hector_state_ != estimator_state_t::reliable) {
      response->success = false;
      response->message = "Cannot change only to HECTOR. HECTOR is not reliable. Current HECTOR state is " + to_string(hector_state_);
      return true;
    }
    gps_state_ = estimator_state_t::inactive;
    RCLCPP_WARN(get_logger(), "Changing odometry to HECTOR only");
  }

  response->success = true;
  response->message = "Done";

  return true;
}
//}

/* resetHectorCallback //{ */
bool Odometry2::resetHectorCallback([[maybe_unused]] const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                    std::shared_ptr<std_srvs::srv::Trigger::Response>                       response) {
  if (!is_initialized_) {
    response->message = "ROS not initialized";
    response->success = false;
    return true;
  }

  std::scoped_lock lock(hector_mutex_);

  // Check if hector might be already in reset mode
  if (hector_state_ == estimator_state_t::restart || hector_state_ == estimator_state_t::restarting) {
    response->message = "Hector in reset mode";
    response->success = false;
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 500, "Hector state: Already in reset mode.");
    return true;
  }

  hector_state_ = estimator_state_t::restart;
  RCLCPP_WARN(get_logger(), "Hector state: Service set hector to the restart.");

  response->message = "Hector set to restart.";
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
  std::scoped_lock lock(odometry_mutex_);

  const auto prev_state = odometry_state_;
  update_odometry_state();

  if (prev_state != odometry_state_)
    publishOdometryDiagnostics();
}
//}

/* odometryDiagnosticsRoutine(); //{ */
void Odometry2::odometryDiagnosticsRoutine() {
  scope_timer      tim(scope_timer_enable_, "odometryDiagnosticsRoutine", get_logger(), scope_timer_throttle_, scope_timer_min_dur_);
  std::scoped_lock lock(odometry_mutex_);

  // publish some diags
  publishOdometryDiagnostics();
}
//}

/* odometryPublisherRoutine(); //{ */
void Odometry2::odometryPublisherRoutine() {
  scope_timer      tim(scope_timer_enable_, "odometryPublisherRoutine", get_logger(), scope_timer_throttle_, scope_timer_min_dur_);
  std::scoped_lock lock(odometry_mutex_, px4_pose_mutex_);

  if (odometry_state_ != odometry_state_t::init && odometry_state_ != odometry_state_t::not_connected &&
      odometry_state_ != odometry_state_t::missing_odometry) {
    // publish odometry
    publishTF();
    publishOdometry();
  } else {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "Odometry publisher: Waiting to set the GPS");
  }
}
//}

/* update_odometry_state//{ */
// the following mutexes have to be locked by the calling function:
// odometry_mutex_
void Odometry2::update_odometry_state() {

  // process the vehicle's state
  switch (odometry_state_) {
    case odometry_state_t::not_connected:
      state_odometry_not_connected();
      break;
    case odometry_state_t::init:
      state_odometry_init();
      break;
    case odometry_state_t::switching:
      state_odometry_switching();
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
    case odometry_state_t::waiting_odometry:
      state_odometry_waiting_odometry();
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
// odometry_mutex_
void Odometry2::state_odometry_not_connected() {

  RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "Odometry state: Waiting to initialize Pixhawk and Control Interface.");

  if (getting_pixhawk_odometry_ && getting_control_interface_diagnostics_) {
    RCLCPP_INFO(get_logger(), "Odometry state: Pixhawk and Control Interface is ready");
    odometry_state_ = odometry_state_t::init;
  }
}
//}

/* state_odometry_init//{ */
// the following mutexes have to be locked by the calling function:
// odometry_mutex_
void Odometry2::state_odometry_init() {

  RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "Odometry state: Initializing odometry module.");

  // Set and handle initial PX4 parameters setting
  if (!set_initial_px4_params_) {
    RCLCPP_INFO(get_logger(), "Odometry state: Setting initial PX4 parameters for GPS.");
    if (setPx4Params(gps_px4_params_int_, gps_px4_params_float_)) {
      last_set_parameters_    = odometry_state_t::gps;
      set_initial_px4_params_ = true;
    } else {
      RCLCPP_ERROR(get_logger(), "Odometry state: Fail to set all PX4 parameters. Will repeat in next cycle.");
      return;
    }
  }

  // Handle static TF initialization
  if (static_tf_broadcaster_ == nullptr) {
    static_tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this->shared_from_this());
    publishStaticTF();
    RCLCPP_INFO(get_logger(), "Odometry state: Publishing static TF");
  }

  // Wait until gps estimator is ready. GPS is required to initalize hector
  std::scoped_lock lock(gps_mutex_);

  if (gps_state_ == estimator_state_t::reliable) {
    odometry_state_ = odometry_state_t::gps;
    return;
  } else {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "Odometry state: Waiting for reliable GPS estimator.");
    return;
  }
}
//}

/* state_odometry_switching//{ */
// the following mutexes have to be locked by the calling function:
// odometry_mutex_
void Odometry2::state_odometry_switching() {

  RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "Odometry state: Switching state");

  // Call land if missing odometry
  if (switching_state_ == odometry_state_t::missing_odometry) {
    odometry_state_ = odometry_state_t::missing_odometry;
    assert(false);
    RCLCPP_ERROR(get_logger(), "Missing odometry in a switch odometry should not happen.");

    return;
  }

  // Update EKF parameters for the new estimator
  if (updateEkfParameters()) {
    // Update successfull
    odometry_state_ = switching_state_;
    return;
  } else {
    // If update fail, repeat
    RCLCPP_WARN(get_logger(), "Fail to set all PX4 parameters for new estimator. Repeat in next cycle.");
  }
}
//}

/* state_odometry_gps//{ */
// the following mutexes have to be locked by the calling function:
// odometry_mutex_
void Odometry2::state_odometry_gps() {

  RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "Odometry state: GPS");

  checkEstimatorReliability();
}
//}

/* state_odometry_hector//{ */
// the following mutexes have to be locked by the calling function:
// odometry_mutex_
void Odometry2::state_odometry_hector() {

  RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "Odometry state: Hector");

  checkEstimatorReliability();
}
//}

/* state_odometry_missing_odometry//{ */
// the following mutexes have to be locked by the calling function:
// odometry_mutex_
void Odometry2::state_odometry_missing_odometry() {

  RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000, "Odometry state: Missing odometry");

  // Call land via control_interface package
  auto future_response = land_client_->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
  RCLCPP_WARN(get_logger(), "Odometry state: Land called.");
  if (future_response.wait_for(std::chrono::seconds(1)) == std::future_status::timeout || future_response.get() == nullptr) {
    RCLCPP_ERROR(get_logger(), "Odometry state: Did not receive land response for 1 second, try again.");
    return;
  } else {
    RCLCPP_INFO(get_logger(), "Odometry state: Land call successfull, switching to waiting state");
    odometry_state_ = odometry_state_t::waiting_odometry;
    return;
  }
}
//}

/* state_odometry_waiting_odometry//{ */
// the following mutexes have to be locked by the calling function:
// odometry_mutex_
void Odometry2::state_odometry_waiting_odometry() {

  RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "Odometry state: Waiting for reliable odometry");

  checkEstimatorReliability();
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
    case estimator_state_t::restart:
      state_gps_restart();
      break;
    case estimator_state_t::restarting:
      state_gps_restarting();
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

  RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "GPS state: Initializing.");

  if (getting_pixhawk_odometry_ && getting_gps_) {
    RCLCPP_INFO(get_logger(), "GPS state: GPS and pixhawk odometry received");
  } else {
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "GPS state: Waiting to receive GPS and odometry from Pixhawk.");
    return;
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

  std::scoped_lock lock(gps_raw_mutex_, px4_pose_mutex_);

  // Check if new GPS message has arrived
  if (gps_last_processed_msg_time_ != gps_last_msg_time_) {
    gps_last_processed_msg_time_ = gps_last_msg_time_;
    // Check gps reliability
    if (!checkGpsReliability()) {
      gps_state_ = estimator_state_t::not_reliable;
      return;
    }
  }

  // Publish TFs and odometry
  publishGpsTF();
  publishGpsOdometry();
}
//}

/* state_gps_not_reliable//{ */
// the following mutexes have to be locked by the calling function:
// gps_mutex_
void Odometry2::state_gps_not_reliable() {

  RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "GPS state: Not reliable.");

  // Check if new GPS message has arrived
  if (gps_last_processed_msg_time_ != gps_last_msg_time_) {
    gps_last_processed_msg_time_ = gps_last_msg_time_;
    // Check gps reliability
    if (checkGpsReliability()) {
      gps_state_ = estimator_state_t::reliable;
      return;
    }
  }

  /*   // Check if new GPS message has arrived */
  /*   if (gps_last_processed_msg_time_ == gps_last_msg_time_) { */
  /*     return; */
  /*   } */
  /*   gps_last_processed_msg_time_ = gps_last_msg_time_; */

  /*   // GPS quality is getting better */
  /*   if (gps_eph_ < gps_eph_max_) { */
  /*     if (c_gps_eph_good_++ >= gps_msg_good_) { */

  /*       RCLCPP_WARN(get_logger(), "GPS quality is good! Reliable."); */
  /*       c_gps_eph_good_ = 0; */
  /*       gps_state_      = estimator_state_t::reliable; */
  /*       return; */

  /*     } else { */
  /*       RCLCPP_WARN(get_logger(), "GPS quality is improving! #%d EPH value: %f", c_gps_eph_good_, gps_eph_); */
  /*     } */
  /*   } else { */
  /*     c_gps_eph_good_ = 0;  // Reset the good message counter */
  /*   } */
}
//}

/* state_gps_inactive//{ */
// the following mutexes have to be locked by the calling function:
// gps_mutex_
void Odometry2::state_gps_inactive() {

  RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "GPS state: Inactive.");
}
//}

/* state_gps_restart//{ */
// the following mutexes have to be locked by the calling function:
// gps_mutex_
void Odometry2::state_gps_restart() {
  assert(false);
  RCLCPP_ERROR(get_logger(), "GPS in restart mode, this should never happen!");
}
//}

/* state_gps_restarting//{ */
// the following mutexes have to be locked by the calling function:
// gps_mutex_
void Odometry2::state_gps_restarting() {
  assert(false);
  RCLCPP_ERROR(get_logger(), "GPS in restart mode, this should never happen!");
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
    case estimator_state_t::inactive:
      state_hector_inactive();
      break;
    case estimator_state_t::restart:
      state_hector_restart();
      break;
    case estimator_state_t::restarting:
      state_hector_restarting();
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

  RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "Hector state: Initializing");

  std::scoped_lock lock(garmin_mutex_);

  if (!getting_hector_) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "Hector state: Waiting to receive hector data.");
    return;
  }

  if (!getting_garmin_) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "Hector state: Waiting to receive garmin data.");
    return;
  }

  std::string temp;
  if (!tf_buffer_->canTransform(fcu_frame_, local_origin_frame_, rclcpp::Time(0), std::chrono::duration<double>(0), &temp)) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "Hector state: Missing TF between ned_origin and frd_fcu frame from GPS.");
    return;
  }

  /* Setup hector_origin based on current gps_position */ /*//{*/
  if (publishing_static_tf_ && getting_pixhawk_odometry_) {
    auto tf             = transformBetween(fcu_frame_, local_origin_frame_);
    pos_orig_hector_[0] = tf.pose.position.x;
    pos_orig_hector_[1] = tf.pose.position.y;
    pos_orig_hector_[2] = 0;
    ori_orig_hector_[0] = tf.pose.orientation.w;
    ori_orig_hector_[1] = tf.pose.orientation.x;
    ori_orig_hector_[2] = tf.pose.orientation.y;
    ori_orig_hector_[3] = tf.pose.orientation.z;
    RCLCPP_INFO(get_logger(), "Hector origin coordinates set - x: %f y: %f z: %f, w: %f, x: %f, y: %f, z: %f", pos_orig_hector_[0], pos_orig_hector_[1],
                pos_orig_hector_[2], ori_orig_hector_[0], ori_orig_hector_[1], ori_orig_hector_[2], ori_orig_hector_[3]);
    hector_tf_setup_ = true;
  } else {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "Hector state: Waiting for pixhawk odometry to initialize tf");
    return;
  }
  /*//}*/

  // Initialize estimator timer
  if (!time_odometry_timer_set_) {
    time_odometry_timer_prev_ = std::chrono::high_resolution_clock::now();
    time_odometry_timer_set_  = true;
  }

  RCLCPP_INFO(get_logger(), "Hector state: Initalizied!");

  // Check if the estimator should be active
  if (!hector_active_) {
    hector_state_ = estimator_state_t::inactive;
    return;
  }
  hector_state_         = estimator_state_t::not_reliable;
  hector_reliable_time_ = std::chrono::system_clock::now();
}
//}

/* state_hector_reliable//{ */
// the following mutexes have to be locked by the calling function:
// hector_mutex_
void Odometry2::state_hector_reliable() {

  RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "Hector state: Reliable");

  std::scoped_lock lock(hector_raw_mutex_, hector_lat_position_mutex_);

  // Check hector reliability
  if (!checkHectorReliability()) {
    hector_state_ = estimator_state_t::not_reliable;
    return;
  }

  updateHectorEstimator();

  // Publish TFs and odometry
  publishHectorTF();
  publishHectorOdometry();
}
//}

/* state_hector_not_reliable//{ */
// the following mutexes have to be locked by the calling function:
// hector_mutex_
void Odometry2::state_hector_not_reliable() {

  RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "Hector state: Not reliable.");

  if (checkHectorReliability()) {
    // Calculate the duration of hector reliability
    std::chrono::duration<double> dt = std::chrono::system_clock::now() - hector_reliable_time_;
    if (dt.count() > hector_reliable_time_limit_) {
      // Hector is reliable for a long time - reliable
      RCLCPP_INFO(get_logger(), "Hector state: Reliable for %f seconds. Switch state to reliable.", hector_reliable_time_limit_);
      hector_state_ = estimator_state_t::reliable;
      return;
    }
  } else {
    hector_state_ = estimator_state_t::restart;
    return;
  }

  updateHectorEstimator();

}  // namespace odometry2
//}

/* state_hector_restart//{ */
// the following mutexes have to be locked by the calling function:
// hector_mutex_
void Odometry2::state_hector_restart() {

  RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "Hector state: Restart.");

  // Check the last hector reset time
  std::chrono::duration<double> dt = std::chrono::system_clock::now() - hector_reset_called_time_;
  if (dt.count() > hector_reset_wait_) {
    RCLCPP_INFO(get_logger(), "Hector state: State change to restarting.");
    hector_state_ = estimator_state_t::restarting;
  } else {
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "Hector state: Waiting for next hector reset availability. Call again in dt: %f",
                         hector_reset_wait_ - dt.count());
  }
}
//}

/* state_hector_inactive//{ */
// the following mutexes have to be locked by the calling function:
// hector_mutex_
void Odometry2::state_hector_inactive() {

  RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "Hector state: Inactive.");
}
//}

/* state_hector_restarting//{ */
// the following mutexes have to be locked by the calling function:
// hector_mutex_
void Odometry2::state_hector_restarting() {

  RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "Hector state: Restarting.");

  std::scoped_lock lock(hector_raw_mutex_, hector_lat_estimator_mutex_, hector_lat_position_mutex_);

  // Reset hector
  auto future_response = reset_hector_client_->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
  RCLCPP_INFO(get_logger(), "Hector state: Hector reset called.");
  if (future_response.wait_for(std::chrono::seconds(hector_reset_response_wait_)) == std::future_status::timeout || future_response.get() == nullptr) {
    RCLCPP_ERROR(get_logger(), "Hector state: Could not reset hector, try again soon.");
    return;
  } else {
    RCLCPP_INFO(get_logger(), "Hector state: Hector node reset successfull.");
  }

  // Reset hector estimator and variables
  hector_lat_estimator_->setState(0, 0);
  hector_lat_estimator_->setState(1, 0);
  hector_lat_estimator_->setState(2, 0);
  hector_lat_estimator_->setState(3, 0);
  hector_lat_estimator_->setState(4, 0);
  hector_lat_estimator_->setState(5, 0);
  time_odometry_timer_set_ = false;

  // Reset hector position, velocity, and orientation
  hector_position_[0] = 0;
  hector_position_[1] = 0;
  hector_position_[2] = 0;

  hector_velocity_[0] = 0;
  hector_velocity_[1] = 0;
  hector_velocity_[2] = 0;

  hector_orientation_[0] = 0;
  hector_orientation_[1] = 0;
  hector_orientation_[2] = 0;
  hector_orientation_[3] = 0;

  // Reset hector raw variables for hector callback
  hector_position_raw_[0]      = 0;
  hector_position_raw_[1]      = 0;
  hector_position_raw_prev_[0] = 0;
  hector_position_raw_prev_[1] = 0;

  c_hector_init_msgs_       = 0;
  hector_tf_setup_          = false;
  hector_reset_called_time_ = std::chrono::system_clock::now();
  getting_hector_           = false;

  RCLCPP_INFO(get_logger(), "Hector state: Hector estimator restarted!");

  hector_state_ = estimator_state_t::init;
}
//}

// | --------------------- Hector methods --------------------- |

/* checkHectorReliability //{*/
// the following mutexes have to be locked by the calling function:
// hector_raw_mutex_
bool Odometry2::checkHectorReliability() {

  // Check hector message interval//{
  std::chrono::duration<double> dt = std::chrono::system_clock::now() - hector_last_msg_time_;

  if (dt.count() > hector_msg_interval_warn_) {
    RCLCPP_WARN(get_logger(), "Hector pose not received for %f seconds.", dt.count());
    if (dt.count() > hector_msg_interval_max_) {
      RCLCPP_WARN(get_logger(), "Hector pose not received for %f seconds. Not reliable.", dt.count());
      return false;
    }
  }
  /*//}*/

  // Detect position jump in consecutive hector poses//{
  if (std::pow(hector_position_raw_[0] - hector_position_raw_prev_[0], 2) > hector_max_position_jump_ ||
      std::pow(hector_position_raw_[1] - hector_position_raw_prev_[1], 2) > hector_max_position_jump_) {
    RCLCPP_WARN(get_logger(), "Jump detected in Hector Slam pose. orig_x: %f, orig_y: %f, x: %f, y: %f . Not reliable", hector_position_raw_prev_[0],
                hector_position_raw_prev_[1], hector_position_raw_[0], hector_position_raw_[1]);
    return false;
  }
  /*//}*/

  // No update in consecutive hector poses -> no features//{
  if (std::pow(hector_position_raw_[0] - hector_position_raw_prev_[0], 2) == 0 && std::pow(hector_position_raw_[1] - hector_position_raw_prev_[1], 2) == 0) {
    RCLCPP_WARN(get_logger(), "Hector does not have any features. orig_x: %f, orig_y: %f, x: %f, y: %f . Not reliable", hector_position_raw_prev_[0],
                hector_position_raw_prev_[1], hector_position_raw_[0], hector_position_raw_[1]);
    return false;
  }
  /*//}*/

  // Detect velocity jump in consecutive hector poses//{
  if (hector_velocity_[0] > hector_max_velocity_ || hector_velocity_[1] > hector_max_velocity_) {
    RCLCPP_WARN(get_logger(), "Hector velocity too large - x: %f, y: %f. Not reliable.", hector_velocity_[0], hector_velocity_[1]);
    return false;
  } /*//}*/

  return true;

} /*//}*/

/* checkGpsReliability //{*/
// the following mutexes have to be locked by the calling function:
// gps_raw_mutex_
// gps_mutex_
bool Odometry2::checkGpsReliability() {

  // Check gps message interval//{
  std::chrono::duration<double> dt = std::chrono::system_clock::now() - gps_last_msg_time_;

  if (dt.count() > gps_msg_interval_warn_) {
    RCLCPP_WARN(get_logger(), "GPS message not received for %f seconds.", dt.count());
    if (dt.count() > gps_msg_interval_max_) {
      RCLCPP_WARN(get_logger(), "GPS message not received for %f seconds. Not reliable.", dt.count());
      /* gps_eph_ = std::numeric_limits<float>::max();  // Set value to max in case GPS stop publishing */
      return false;
    }
  }
  /*//}*/

  /* Check gps quality//{*/
  if (gps_state_ == estimator_state_t::reliable) {

    if (gps_eph_ >= gps_eph_max_) {
      if (c_gps_eph_err_++ >= gps_msg_err_) {  // GPS quality is lower for a specific number of consecutive messages
        RCLCPP_WARN(get_logger(), "GPS quality is too low! Not reliable");
        c_gps_eph_err_ = 0;
        return false;

      } else {
        RCLCPP_WARN(get_logger(), "GPS quality is too low! #%d/#%d EPH value: %f", c_gps_eph_err_, gps_msg_err_, gps_eph_);
        return true;
      }
    } else {
      c_gps_eph_err_ = 0;  // Reset the bad message counter
      return true;
    }
  }
  /*//}*/

  // GPS quality is getting better
  if (gps_state_ == estimator_state_t::not_reliable) {
    if (gps_eph_ < gps_eph_max_) {
      if (c_gps_eph_good_++ >= gps_msg_good_) {

        RCLCPP_WARN(get_logger(), "GPS quality is good! Reliable.");
        c_gps_eph_good_ = 0;
        return true;

      } else {
        RCLCPP_WARN(get_logger(), "GPS quality is improving! #%d EPH value: %f", c_gps_eph_good_, gps_eph_);
        return false;
      }
    } else {
      c_gps_eph_good_ = 0;  // Reset the good message counter
      return false;
    }
  }
  assert(false);
  RCLCPP_ERROR(get_logger(), "This should not happen.");
} /*//}*

// --------------------------------------------------------------
// |                           Publish                          |
// --------------------------------------------------------------

// PublishStaticTF //{*/
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

  tf.header.frame_id         = local_origin_frame_;
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

  tf_local_origin_to_ned_origin_frame_ = tf;

  static_tf_broadcaster_->sendTransform(v_transforms);

  publishing_static_tf_ = true;
}
/*//}*/

/* publishGpsTF //{ */
// the following mutexes have to be locked by the calling function:
// px4_pose_mutex_
void Odometry2::publishGpsTF() {
}
//}

/* publishGpsOdometry //{ */
// the following mutexes have to be locked by the calling function:
// px4_pose_mutex_
void Odometry2::publishGpsOdometry() {
}
//}

/* publishTF //{ */
// the following mutexes have to be locked by the calling function:
// px4_pose_mutex_
void Odometry2::publishTF() {

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

/* publishOdometry //{ */
// the following mutexes have to be locked by the calling function:
// px4_pose_mutex_
void Odometry2::publishOdometry() {

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
// the following mutexes have to be locked by the calling function:
// hector_lat_position_mutex_
void Odometry2::publishHectorTF() {

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
// the following mutexes have to be locked by the calling function:
// hector_lat_position_mutex_
void Odometry2::publishHectorOdometry() {

  std::scoped_lock lock(timestamp_mutex_);

  std::string temp;
  if (!tf_buffer_->canTransform(ned_origin_frame_, hector_origin_frame_, rclcpp::Time(0), std::chrono::duration<double>(0), &temp)) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "Hector state: Missing hector origin frame - waiting for hector initialization.");
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
  px4_msgs::msg::VehicleVisualOdometry hector_odometry_msg;

  // Compensate time with timestamp message from px
  std::chrono::duration<long int, std::nano> diff = std::chrono::high_resolution_clock::now() - time_sync_time_;
  hector_odometry_msg.timestamp                   = timestamp_ / 1000 + diff.count() / 1000;
  hector_odometry_msg.timestamp_sample            = timestamp_ / 1000 + diff.count() / 1000;

  hector_odometry_msg.x = hector_pos_tf.point.x;
  hector_odometry_msg.y = hector_pos_tf.point.y;
  /* hector_odometry_msg.x = hector_pos_tf.point.x - px4_position_[0]; // Correcting hector position to the GPS position in case of the switch TODO: do not use
   * position but rather get tf and use the value from it - do not need to lock mutex */
  /* hector_odometry_msg.y = hector_pos_tf.point.y - px4_position_[1]; */
  hector_odometry_msg.z = hector_pos_tf.point.z;

  hector_odometry_msg.vx = hector_vel_tf.vector.x;
  hector_odometry_msg.vy = hector_vel_tf.vector.y;
  hector_odometry_msg.vz = hector_vel_tf.vector.z;

  hector_odometry_msg.rollspeed  = NAN;
  hector_odometry_msg.pitchspeed = NAN;
  hector_odometry_msg.yawspeed   = NAN;

  hector_odometry_msg.q[0] = hector_orientation_[0];
  hector_odometry_msg.q[1] = hector_orientation_[1];
  hector_odometry_msg.q[2] = hector_orientation_[2];
  hector_odometry_msg.q[3] = hector_orientation_[3];

  std::fill(hector_odometry_msg.q_offset.begin(), hector_odometry_msg.q_offset.end(), NAN);
  std::fill(hector_odometry_msg.pose_covariance.begin(), hector_odometry_msg.pose_covariance.end(), NAN);
  std::fill(hector_odometry_msg.velocity_covariance.begin(), hector_odometry_msg.velocity_covariance.end(), NAN);

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
  hector_odometry_publisher_->publish(hector_odometry_msg);

  // Publish hector in the correct RVIZ orientation visualization
  nav_msgs::msg::Odometry rviz_msg;

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
// odometry_mutex_
void Odometry2::publishOdometryDiagnostics() {
  fog_msgs::msg::OdometryDiagnostics msg;

  msg.header.stamp = get_clock()->now();

  msg.odometry_state            = to_msg(odometry_state_);
  msg.getting_pixhawk_odometry  = getting_pixhawk_odometry_;
  msg.getting_control_interface = getting_control_interface_diagnostics_;

  odometry_diagnostics_publisher_->publish(msg);
}
//}

/* publishGpsDiagnostics //{ */
// the following mutexes have to be locked by the calling function:
// gps_mutex_
void Odometry2::publishGpsDiagnostics() {
  fog_msgs::msg::EstimatorDiagnostics msg;

  msg.header.stamp = get_clock()->now();

  msg.estimator_state = to_msg(gps_state_);

  gps_diagnostics_publisher_->publish(msg);
}
//}

/* publishHectorDiagnostics //{ */
// the following mutexes have to be locked by the calling function:
// hector_mutex_
void Odometry2::publishHectorDiagnostics() {
  fog_msgs::msg::EstimatorDiagnostics msg;

  msg.header.stamp = get_clock()->now();

  msg.estimator_state = to_msg(hector_state_);

  hector_diagnostics_publisher_->publish(msg);
}
//}

/* updateHectorEstimator //{ */
// the following mutexes have to be locked by the calling function:
// hector_mutex_
// hector_raw_mutex_
// hector_lat_position_mutex_
void Odometry2::updateHectorEstimator() {

  std::scoped_lock lock(hector_lat_estimator_mutex_, garmin_mutex_);

  // Calculate time since last estimators update
  std::chrono::time_point<std::chrono::high_resolution_clock> time_now = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double, std::milli>                   dt       = (time_now - time_odometry_timer_prev_) / 1000;

  if (dt.count() <= 0) {
    RCLCPP_WARN(get_logger(), "Odometry timer dt=%f is lower than zero, skipping estimator update.", dt.count());
    return;
  }

  time_odometry_timer_prev_ = time_now;

  // Latitude estimator update and predict
  hector_lat_estimator_->doCorrection(hector_position_raw_[0], hector_position_raw_[1], LAT_HECTOR);
  hector_lat_estimator_->doPrediction(0.0, 0.0, dt.count());

  // Altitude estimator update and predict
  garmin_alt_estimator_->doCorrection(garmin_measurement_, ALT_GARMIN);
  garmin_alt_estimator_->doPrediction(0.0, dt.count());

  // Obtain positions and velocities from estimators
  double pos_x, pos_y, pos_z, vel_x, vel_y, vel_z;

  hector_lat_estimator_->getState(0, pos_x);
  hector_lat_estimator_->getState(1, pos_y);
  hector_lat_estimator_->getState(2, vel_x);
  hector_lat_estimator_->getState(3, vel_y);

  garmin_alt_estimator_->getState(0, pos_z);
  garmin_alt_estimator_->getState(1, vel_z);

  // Set position and velocity for hector further usage
  hector_position_[0] = pos_x;
  hector_position_[1] = pos_y;
  hector_position_[2] = pos_z - std::abs(garmin_offset_);

  hector_velocity_[0] = vel_x;
  hector_velocity_[1] = vel_y;
  hector_velocity_[2] = vel_z;

  // Get orientation from px orientation
  std::string temp;
  if (!tf_buffer_->canTransform(ned_origin_frame_, frd_fcu_frame_, rclcpp::Time(0), std::chrono::duration<double>(0), &temp)) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "Hector state: Missing ned_origin to frd_fcu - waiting for px odometry initialization.");
    return;
  }

  auto transform_stamped = tf_buffer_->lookupTransform(ned_origin_frame_, frd_fcu_frame_, rclcpp::Time(0));

  hector_orientation_[0] = transform_stamped.transform.rotation.w;
  hector_orientation_[1] = transform_stamped.transform.rotation.x;
  hector_orientation_[2] = transform_stamped.transform.rotation.y;
  hector_orientation_[3] = transform_stamped.transform.rotation.z;
}
//}


// | -------------------------- Utils ------------------------- |

/* checkEstimatorReliability//{ */
// the following mutexes have to be locked by the calling function:
// odometry_mutex_
void Odometry2::checkEstimatorReliability() {

  std::scoped_lock lock(gps_mutex_, hector_mutex_);

  odometry_state_t prev_state = odometry_state_;

  if (gps_state_ == estimator_state_t::reliable && hector_state_ == estimator_state_t::reliable) {
    switching_state_ = odometry_state_t::gps;
  } else if (gps_state_ == estimator_state_t::reliable && hector_state_ != estimator_state_t::reliable) {
    switching_state_ = odometry_state_t::gps;
  } else if (gps_state_ != estimator_state_t::reliable && hector_state_ == estimator_state_t::reliable) {
    switching_state_ = odometry_state_t::hector;
  } else if (gps_state_ != estimator_state_t::reliable && hector_state_ != estimator_state_t::reliable) {
    // Check if the node is already waiting for a reliable estimator
    if (odometry_state_ == odometry_state_t::waiting_odometry) {
      return;
    } else {
      odometry_state_ = odometry_state_t::missing_odometry;
      return;
    }
  } else {
    switching_state_ = odometry_state_t::invalid;
    assert(false);
    RCLCPP_ERROR(get_logger(), "This should not happen.");
  }

  // Check if odometry state is planned to change
  if (prev_state != switching_state_) {
    odometry_state_ = odometry_state_t::switching;
  }
}
//}

/* updateEkfParameters//{ */
// the following mutexes have to be locked by the calling function:
// odometry_mutex_
bool Odometry2::updateEkfParameters() {

  std::vector<px4_int> px4_params_int;

  if (switching_state_ == odometry_state_t::gps) {
    if (last_set_parameters_ == odometry_state_t::gps) {
      // Parameters are already set up from before, return right away
      RCLCPP_INFO(get_logger(), "Odometry state: Do not have to update parameters for GPS, already set up.");
      return true;
    }
    if (setPx4Params(gps_px4_params_int_, gps_px4_params_float_)) {
      last_set_parameters_ = odometry_state_t::gps;
      return true;
    }
  } else if (switching_state_ == odometry_state_t::hector) {
    if (last_set_parameters_ == odometry_state_t::hector) {
      RCLCPP_INFO(get_logger(), "Odometry state: Do not have to update parameters for HECTOR, already set up.");
      // Parameters are already set up from before, return right away
      return true;
    }
    if (setPx4Params(hector_px4_params_int_, hector_px4_params_float_)) {
      last_set_parameters_ = odometry_state_t::hector;
      return true;
    }
  } else {
    assert(false);
    RCLCPP_ERROR(get_logger(), "Invalid switching state, this should never happen!");
  }
  return false;
}
//}

/* transformBetween //{ */
geometry_msgs::msg::PoseStamped Odometry2::transformBetween(std::string& frame_from, std::string& frame_to) {
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

/*setPx4Params//{*/
bool Odometry2::setPx4Params(const std::vector<px4_int>& params_int, const std::vector<px4_float>& params_float) {

  if (!uploadPx4Parameters(std::make_shared<fog_msgs::srv::SetPx4ParamInt::Request>(), params_int, set_px4_param_int_) ||
      !uploadPx4Parameters(std::make_shared<fog_msgs::srv::SetPx4ParamFloat::Request>(), params_float, set_px4_param_float_)) {
    return false;
  }
  return true;
}
/*//}*/

/* uploadPx4Parameters //{ */
template <typename T, typename U, typename V>
bool Odometry2::uploadPx4Parameters(const std::shared_ptr<T>& request, const std::vector<U>& param_array, const V& service_client) {
  for (const auto item : param_array) {
    request->param_name = std::get<0>(item);
    request->value      = std::get<1>(item);
    RCLCPP_INFO(get_logger(), "Setting %s, value: %f", std::get<0>(item).c_str(), (float)std::get<1>(item));
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
bool Odometry2::checkPx4ParamSetOutput(const std::shared_future<T> f) {
  assert(f.valid());
  if (f.wait_for(std::chrono::seconds(px4_param_set_timeout_)) == std::future_status::timeout || f.get() == nullptr) {
    RCLCPP_ERROR(get_logger(), "Cannot set the parameter %s with message: %s", f.get()->param_name.c_str(), f.get()->message.c_str());
    return false;
  } else {
    RCLCPP_INFO(get_logger(), "Parameter %s has been set to value: %f", f.get()->param_name.c_str(), (float)f.get()->value);
    return true;
  }
}
//}

/* new_cbk_grp() method //{ */
// just a util function that returns a new mutually exclusive callback group to shorten the call
rclcpp::CallbackGroup::SharedPtr Odometry2::new_cbk_grp() {
  const rclcpp::CallbackGroup::SharedPtr new_group = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  callback_groups_.push_back(new_group);
  return new_group;
}
//}

/*isValidGate() //{*/
bool Odometry2::isValidGate(const double& value, const double& min_value, const double& max_value, const std::string& value_name) {

  // Min value check
  if (value < min_value) {
    if (value_name != "") {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "%s value %f < %f is not valid.", value_name.c_str(), value, min_value);
    }
    return false;
  }

  // Max value check
  if (value > max_value) {
    if (value_name != "") {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "%s value %f > %f is not valid.", value_name.c_str(), value, max_value);
    }
    return false;
  }

  return true;
}
/*//}*/

/* //{ isValidType */
bool Odometry2::isValidType(const fog_msgs::msg::OdometryType& type) {

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
